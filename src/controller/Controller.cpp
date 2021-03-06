#include "Controller.hpp"
// #include "PControl.hpp"

void* startThreadCalculateMovement(void *something);
void* startThreadBuildFormation(void *something);
void* startThreadShutdown(void *something);
void* startThreadRotation(void *something);
static bool closeToTarget(Position6DOF position1, Position6DOF position2, double range);

/**
 * Constructor of Controller.
 * Sets all control variable on false, sets up all the Ros functions, and
 * initializes helper variables.
 */
Controller::Controller()
{
	// All control variables are set to zero
	this->constructionDone = false;
	this->shutdownStarted = false;  // shutdown not started
	this->landFinished = false;
	this->receivedQuadcopters = false; // received no quadcopters
	this->receivedFormation = false;
	this->buildFormationFinished = false; // has not built formation
	this->formation = new Formation();
	this->receivedTrackingArea = false;
	this->rotationInProcess = false;
	this->buildFormationStarted = false;
	this->tBuildFormation = 0;
	this->tCalculateMovement = 0;
	this->tShutdownFormation = 0;
	this->tRotation = 0;

	for (int i = 0; i < MAX_NUMBER_QUADCOPTER; i++) {
		tracked[i] = false; // Initialize tracked (no quadcopter is tracked at
		                    // the beginning)
		receivedQuadStatus[i] = false;
		this->emergencyShutdown[i] = false;
		timeLastCurrent[i] = 0;
		// this->quadcopterStatus[i] = QuadcopterControl();
		if (!USE_BATTERY_INPUT) {
			quadcopterStatus[i].getQuadcopterThrust().setWithoutBatteryValue();
		}
		this->thrustHelp[i] = quadcopterStatus[i].getQuadcopterThrust().getStart();
		// this->quadcopterMovementStatus[i] = CALCULATE_NONE;
		pitch_stab[i] = 0;
		roll_stab[i] = 0;
		yaw_stab[i] = 0;
		thrust_stab[i] = 0;
		battery_status[i] = 0;
		this->batteryStatusCounter[i] = 0;
		this->batteryStatusSum[i] = 0;
		this->baro[i] = 0;
		this->baroTarget[i] = this->baro[i] + BARO_OFFSET;
	}


	this->controlThrust = new PDIControl(AMPLIFICATION_THRUST_P_POS, AMPLIFICATION_THRUST_P_NEG, AMPLIFICATION_THRUST_D,
	                                     AMPLIFICATION_THRUST_I, THRUST_OFFSET);
	this->controlRoll = new PControl(AMPLIFICATION_R, RP_OFFSET);
	this->controlPitch = new PControl(AMPLIFICATION_P, RP_OFFSET);
	this->controlYawrate = new PControl(AMPLIFICATION_Y, Y_OFFSET);

	ROS_INFO("CONTROLLER: ROS stuff setting up");
	/*
	 * NodeHandle is the main access point to communications with the ROS
	 * system.
	 * The first NodeHandle constructed will fully initialize this node, and the
	 * last
	 * NodeHandle destructed will close down the node.
	 */
	// Subscriber
	// Subscriber for the MoveFormation data of the Quadcopters (1000 is the
	// max. buffered messages)
	this->MoveFormation_sub = this->n.subscribe("MoveFormation", 1000, &Controller::moveFormationCallback, this);
	// Subscriber for the SetFormation data of the Quadcopters (100 is the max.
	// buffered messages)
	this->SetFormation_sub = this->n.subscribe("SetFormation", 100, &Controller::setFormationCallback, this);
	// Subscriber for the quadcopter status data of the Quadcopters (1000 is the
	// max. buffered messages)
	this->System_sub = this->n.subscribe("System", 1000, &Controller::systemCallback, this);

	// Service
	// Service for BuildFormation and Shutdown
	this->BuildForm_srv  = this->n.advertiseService("BuildFormation", &Controller::startBuildFormation, this);
	this->Shutdown_srv = this->n.advertiseService("Shutdown", &Controller::shutdown, this);
	this->QuadID_srv = this->n.advertiseService("SetQuadcopters", &Controller::setQuadcopters, this);
	this->Rotation_srv = this->n.advertiseService("Rotation", &Controller::rotateFormation, this);

	// Publisher
	// Publisher of Message to API
	this->Message_pub = this->n.advertise<api_application::Message>("Message", 100);

	// Client
	this->Announce_client = this->n.serviceClient<api_application::Announce>("Announce");
	this->Shutdown_client = this->n.serviceClient<control_application::Shutdown>("Shutdown");

	this->timeOffsetOutput = getNanoTime();
	this->timeDurationMoveup = getNanoTime();
	this->timeOffsetChangeThrust = getNanoTime();
	ROS_INFO("CONTROLLER: Constructing done");
	this->constructionDone = true;
}

/**
 * Initialization after the system has been started globally.
 * Controller announces itself to API to get a global ID.
 */
void Controller::initialize()
{
	api_application::Announce srv;
	srv.request.type = 2;
	if (Announce_client.call(srv)) {
		this->senderID = srv.response.id;
	}
	ROS_INFO("CONTROLLER: Initialize done");
}

/**
 * Setter for the Tracking Area.
 * Executed by matlab modul.
 */
void Controller::setTrackingArea(TrackingArea area)
{
	this->trackingArea = area;
	this->receivedTrackingArea = true;
}

/**
 * Calculates the new Targets considering the previous targets and the formation
 * movement vector (without orientation right now)
 */
void Controller::setTargetPosition()
{
	long int currentTime = getNanoTime();
	Position6DOF newTarget;
	int formationAmount = this->formation->getAmount();
	// Iterate over all quadcopters in formation and set new target considering
	// old target and formation Movement
	for (int i = 0; i < formationAmount; i++) {
		this->listTargetsMutex.lock();
		Position6DOF latestTarget = this->listTargets[i].back();
		double targetOld[3];
		for (int k = 0; k < 3; k++) {
			targetOld[k] = latestTarget.getPosition()[k];
		}
		this->listTargetsMutex.unlock();
		double targetNew[3];
		this->formationMovementMutex.lock();
		targetNew[0] = targetOld[0] + this->formationMovement.back()[0];
		targetNew[1] = targetOld[1] + this->formationMovement.back()[1];
		targetNew[2] = targetOld[2] + this->formationMovement.back()[2];
		this->formationMovementMutex.unlock();
		// Check if new position would be in tracking area
		Vector vector = Vector(targetNew[0],targetNew[1],targetNew[2]);
		/*if(!this->trackingArea.contains(vector))	// TODO carina
		 *    {
		 *     std::string message("Formation Movement is invalid. Quadcopter %i
		 * would leave Tracking Area.\n", i);
		 *     ROS_INFO("Warning:Formation Movement is invalid. Quadcopter %i
		 * would leave Tracking Area.",i);
		 *     emergencyRoutine(message);
		 *     return;
		 *    }*/
		newTarget.setPosition(targetNew);
		newTarget.setTimestamp(currentTime);
		this->listTargetsMutex.lock();
		this->listTargets[i].push_back(newTarget);
		// Trim list of Targets to 30 elements
		while (this->listTargets[i].size() > 30) {
			// Remove oldest elements
			this->listTargets[i].erase(this->listTargets[i].begin());
		}
		this->listTargetsMutex.unlock();
	}
}

/**
 * Updating the current Position of the crazyflies.
 * Calles by the position modul with new/current position data.
 * Number of new positions can vary accordingly to the newly calculated
 * positions.
 */
void Controller::updatePositions(std::vector<Vector> positions, std::vector<int> ids, std::vector<int> updates)
{
	if (!receivedQuadcopters || !receivedFormation) {
		ROS_DEBUG("CONTROLLER: No SetQuadcopters or no SetFormation");
		return;
	}
	/* Save position vectors */
	int i = 0;
	int id = 0;
	long int currentTime = getNanoTime();
	for (std::vector<Vector>::iterator it = positions.begin(); it != positions.end(); ++it, i++) {
		if (i >= ids.size()) {
			ROS_DEBUG("CONTROLLER: Id array is too small");
			break;
		}
		// ROS_DEBUG("Id at Update position is %i", ids[i]);
		id = getLocalId(ids[i]);
		// ROS_DEBUG("Id local is %i", id);
		if (id == INVALID) {
			ROS_DEBUG("CONTROLLER: Invalid id %i", id);
			continue;
		}


		// Check if new positions are valid.
		if (it->isValid()) {
			currentTime = getNanoTime();
			this->timeLastCurrent[id] = currentTime;
			Position6DOF newPosition = Position6DOF (it->getV1(), it->getV2(), it->getV3());
			newPosition.setTimestamp(currentTime);
			/* Quadcopter is tracked */
			bool trackedLocal = this->tracked[id];
			if (trackedLocal == false) {
				/* Quadcopter has not been tracked before, therefore set tracked
				 * to true
				 *    and switch to Stabilize-Status when the qc was in starting
				 * process
				 *    or qc has left tracking area and has now entered the
				 * tracking area again*/
				unsigned int quadStatus = this->quadcopterMovementStatus[id];
				bool emergencyLanding = (quadStatus == CALCULATE_LAND) && this->emergencyShutdown[id] &&
				                        this->shutdownStarted == false;
				// if(quadStatus == CALCULATE_START || quadStatus ==
				// CALCULATE_NONE || emergencyLanding)
				if (quadStatus != CALCULATE_LAND || emergencyLanding) {
					ROS_DEBUG("CONTROLLER: Stabilizing now %i", id);
					this->quadcopterMovementStatus[id] = CALCULATE_STABILIZE;
					this->timeDurationMoveup = getNanoTime();
				}
			}
			this->tracked[id] = true;
			control_application::quadcopter_is_tracked msg;
			msg.is_tracked = true;
			Tracked_pub[id].publish(msg);
			this->listPositionsMutex.lock();
			this->listPositions[id].push_back(newPosition);
			this->listPositionsMutex.unlock();
		} else {
			ROS_DEBUG("CONTROLLER: Invalid position of %i", id);
		}
		this->listPositionsMutex.lock();
		// Trim Position list to 30 elements.
		while (this->listPositions[id].size() > 30) {
			this->listPositions[id].erase(this->listPositions[id].begin());     //
			                                                                    // Remove
			                                                                    // oldest
			                                                                    // elements
		}
		this->listPositionsMutex.unlock();
	}
}

/**
 * Creates all the Ros messages for the movement of each quadcopter and sends
 * this
 * to the quadcopter modul
 */
void Controller::sendMovementAll()
{
	// Creates a message for each quadcopter movement and sends it via Ros
	control_application::quadcopter_movement msg;

	long int currentTime = getNanoTime();
	std::vector<MovementQuadruple> newListElement;
	for (int i = 0; i < currentMovement.size(); i++) {
		unsigned int quadStatus = this->quadcopterMovementStatus[i];
		// Check if the qc movement values are in the allowed range.
		switch (quadStatus) {
		case CALCULATE_START:
			this->currentMovement[i].checkQuadruple(
			    quadcopterStatus[i].getQuadcopterThrust().getStartMax(), ROLL_MAX, PITCH_MAX, YAWRATE_MAX);
			break;

		case CALCULATE_LAND:
			break;

		default:
			this->currentMovement[i].checkQuadruple(
			    quadcopterStatus[i].getQuadcopterThrust().getMax(), ROLL_MAX, PITCH_MAX, YAWRATE_MAX);
			break;
		}
		// msg.thrust = 31000;	// JUST FOR TESTING
		// msg.roll = 0.0;	// JUST FOR TESTING
		// msg.pitch = 0.0;	// JUST FOR TESTING
		// msg.yaw = 10.0;	// JUST FOR TESTING
		msg.thrust = this->currentMovement[i].getThrust();
		msg.roll = this->currentMovement[i].getRoll();
		msg.pitch = this->currentMovement[i].getPitch();
		msg.yaw = this->currentMovement[i].getYawrate();
		// ROS_INFO("Roll %f, pitch %f, yaw %f", msg.roll, msg.pitch, msg.yaw);
		this->Movement_pub[i].publish(msg);

		// Trim list of sent movement data to a defined value
		while (this->listSentQuadruples[i].size() > MAX_SAVED_SENT_QUADRUPLES) {
			// Remove oldest elements
			this->listSentQuadruples[i].erase(this->listSentQuadruples[i].begin());
		}
		this->listSentQuadruples[i].push_back(this->currentMovement[i]);    //
		                                                                    // Save
		                                                                    // Element
	}
	// ROS_DEBUG("Send Movement all");
}

/**
 * Creates all the Ros messages for the movement of each quadcopter and sends
 * this
 * to the quadcopter modul
 */
void Controller::sendMovement(int internId)
{
	// Creates a message for each quadcopter movement and sends it via Ros
	control_application::quadcopter_movement msg;

	long int currentTime = getNanoTime();
	std::vector<MovementQuadruple> newListElement;
	unsigned int quadStatus = this->quadcopterMovementStatus[internId];
	// Check if the qc movement values are in the allowed range.
	if (quadStatus == CALCULATE_START) {
		this->currentMovement[internId].checkQuadruple(
		    quadcopterStatus[internId].getQuadcopterThrust().getStartMax(), ROLL_MAX, PITCH_MAX, YAWRATE_MAX);
	} else if (quadStatus == CALCULATE_LAND) {
	} else {
		this->currentMovement[internId].checkQuadruple(
		    quadcopterStatus[internId].getQuadcopterThrust().getMax(), ROLL_MAX, PITCH_MAX, YAWRATE_MAX);
	}
	msg.thrust = this->currentMovement[internId].getThrust();
	msg.roll = this->currentMovement[internId].getRoll();
	msg.pitch = this->currentMovement[internId].getPitch();
	msg.yaw = this->currentMovement[internId].getYawrate();
	this->Movement_pub[internId].publish(msg);
	// Trim list of sent movement data to a defined value
	while (this->listSentQuadruples[internId].size() > MAX_SAVED_SENT_QUADRUPLES) {
		// Remove oldest elements
		this->listSentQuadruples[internId].erase(this->listSentQuadruples[internId].begin());
	}
	// Save Element (TODO only if not too young, in calculateMovement())
	this->listSentQuadruples[internId].push_back(this->currentMovement[internId]);
}

/**
 * Controlls the behaviour of the quadcopters. Each quadcopter has a status
 * according to which different functions are
 * called to set/calculate the movement data considering different situations of
 * the quadcopters.
 * If the quadcopter is out of tracking area or has a low battery status,
 * checkInput will detect the problem and
 * react respectivly.
 * After calculating the movement data considering the quadcopter status the
 * data is sent to the quadcopter.
 * Routine runs till shutdown formaiton is called.
 */
void Controller::calculateMovement()
{
	ROS_INFO("CONTROLLER: Calculation started");
	int waitCounter = 0;
	while ((!this->receivedQuadcopters)) {
		usleep(TIME_WAIT_FOR_QUADCOPTERS);
		waitCounter++;
		if (waitCounter > 100) {
			ROS_ERROR("CONTROLLER: No SetQuadcopters received in time");
			return;
		}
	}
	int numberOfLanded = 0; // Keep track of the number of quadcopters landed
	int amount = quadcopterMovementStatus.size();
	bool end;   // As long as the land process isn't finished, we calculate new
	            // data
	if (this->receivedFormation) {
		/*
		 * When the number of quadcopters landed exceeds the number of
		 * quadcopters
		 * in the formation, the land process is finished.
		 */
		amount = 1; // FIXME
		// end = numberOfLanded >= this->formation->getAmount();
		end = numberOfLanded >= 1;
	} else {
		end = false;
	}
	long int calculateMovementStarted = getNanoTime();
	long int timerCalculateMovement = getNanoTime();
	int loopCounter = 0;
	long int loopCounterTime = getNanoTime();
	while (!end) {
		calculateMovementStarted = getNanoTime();
		// Iterate over the total number of quadcopters as long as the land
		// process isn't finished
		for (int i = 0; (i < amount) && (!end); i++) {
			// Check if there is enough data to check input for correct data.
			bool enoughData = this->receivedFormation && this->receivedQuadcopters;
			unsigned int quadStatus = this->quadcopterMovementStatus[i];
			if (quadStatus == CALCULATE_LAND || quadStatus == CALCULATE_STABILIZE) {
				if (enoughData) {
					// Check if quadcopter is in tracked and/or the battery
					// status is sufficiently high.
					checkInput(i);
				}
			}

			// Calculate movement data accordingly to the quadcopter
			// status/state.
			switch (quadStatus) {
			case CALCULATE_NONE:
				dontMove(i);
				break;
			case CALCULATE_START:
				// ROS_INFO("Start %i", i);
				moveUp(i);
				// stabilize( i );
				break;
			case CALCULATE_STABILIZE:
				stabilize(i);
				break;
			case CALCULATE_HOLD:
				ROS_INFO("CONTROLLER: Hold %i", i);
				hold(i);
				break;
			case CALCULATE_MOVE:
				/* TODO */
				ROS_INFO("Move %i", i);
				break;
			case CALCULATE_LAND:
				if (numberOfLanded > 1 && i == 0) {
					ROS_INFO("CONTROLLER: Landed in Calc: %i", numberOfLanded);
				}
				land(i, &numberOfLanded);
				break;
			default:
				ROS_INFO("CONTROLLER: QC in default status: %i", i);
				break;
			}
			// Check if land process is finished and set control variable
			// accordingly
			if (this->receivedFormation) {
				// end = numberOfLanded >= this->formation->getAmount();
				end = numberOfLanded >= 1; // FIXME
				// ROS_DEBUG("Formation now set");
			} else {
				end = false;
			}
			this->landFinished = end;
		}
		sendMovementAll(); // FIXME
		// Make sure the calculation of the movement data is restricted to a
		// certain rate.
		timerCalculateMovement = getNanoTime();
		long int timeToWait =
		    ((1000000000 / LOOPS_PER_SECOND) - (timerCalculateMovement - calculateMovementStarted)) / 1000;
		long int timeOverhead = 10000;  // in us
		if (timeToWait > timeOverhead) {
			usleep(timeToWait);
		} else {
			// ROS_ERROR("Calculate was too slow: %ld", timeToWait);
		}
		// ROS_INFO("Loop took: %ld ns", (getNanoTime() -
		// calculateMovementStarted));
		loopCounter++;
		if (loopCounter == LOOPS_PER_SECOND) {
			long int current = getNanoTime();
			ROS_DEBUG("CONTROLLER: Time of %i loops: %ld", LOOPS_PER_SECOND, current - loopCounterTime);
			loopCounter = 0;
			loopCounterTime = getNanoTime();
		}
	}
	ROS_ERROR("CONTROLLER: Calculate Movement terminated/n/n!");
}

/**
 * Builds Formation by starting one quadcopter after another, finding the right
 * position and then
 * inclining a little to avoid collisions. So there is a "being tracked" and
 * "moving" level, and a "standing still"
 * at the right position level.
 */
void Controller::buildFormation()
{
	for (int i = 0; i < 1; i++) {
		this->quadcopterMovementStatus[i] = CALCULATE_STABILIZE;
	}
	/*ROS_INFO("Service buildFormation has been called");
	 *    //Check if Formation and Quadcopters have been set and build formation
	 * can be started
	 *    bool notEnoughData = !receivedQuadcopters || !receivedFormation;
	 *    int counter = 0;
	 *    while( notEnoughData )
	 *    {
	 *     ROS_ERROR("Not enough data to build Formation. Waiting to receive
	 * data");
	 *     usleep(TIME_WAIT_FOR_DATA);
	 *     notEnoughData = !receivedQuadcopters || !receivedFormation ;
	 *     counter++;
	 *     if(counter > 100)
	 *     {
	 *         ROS_ERROR("Not enough data to build Formation. Terminating now");
	 *         return;
	 *     }
	 *    }
	 *
	 *    //Get the formation Positions and the distance.
	 *    int formationAmount = this->formation->getAmount();
	 *    ROS_INFO("Formation Amount %i", formationAmount);
	 *    Position6DOF formPos[formationAmount];
	 *    for( int i = 0; i < formationAmount; i++)
	 *    {
	 *     formPos[i] = this->formation->getFormationPosition()[i];
	 *    }
	 *    double distance = this->formation->getDistance();
	 *
	 *    //Pointer to the first tracked quadcopter
	 *    double first[3];
	 *    //Start one quadcopter after another
	 *    for(int i = 0; i < formationAmount; i++)
	 *    {
	 *     ROS_INFO("Starting QC %i", i);
	 *     //Starting/ Inclining process
	 *     bool shutdown = this->shutdownStarted;
	 *     //If Shutdown has been called, abort. Otherwise start Starting
	 * process
	 *     if(shutdown)
	 *     {
	 *         return;
	 *     }
	 *     else
	 *     {
	 *         this->quadcopterMovementStatus[i] = CALCULATE_START;
	 *         this->timeDurationMoveup = getNanoTime();
	 *     }
	 *     //Calculate Target Position of current qc using formation positions
	 * and the formation distance
	 *     double pos[3];
	 *     double target[3];
	 *     for(int k = 0; k < 3; k++)
	 *     {
	 *         pos[k] = formPos[i].getPosition()[k];
	 *         target[k] = pos[k] * distance;
	 *     }
	 *     unsigned int quadStatus = this->quadcopterMovementStatus[i];
	 *     //As long as the quadcopter isn't tracked, incline
	 *     while(quadStatus == CALCULATE_START)
	 *     {
	 *             quadStatus = this->quadcopterMovementStatus[i];
	 *         usleep(TIME_WAIT_FOR_TRACKED);
	 *         //If Shutdown has been called, abort.
	 *         shutdown = this->shutdownStarted;
	 *         if(shutdown)
	 *         {
	 *             ROS_INFO("Shutdown in BuildFormation");
	 *             return;
	 *         }
	 *     }
	 *     ROS_INFO("Tracked");
	 *     //If this is the first tracked quadcopter set it as a reference point
	 * for all the others
	 *     if( i == 0)
	 *     {
	 *         ROS_INFO("First one");
	 *         this->listPositionsMutex.lock();
	 *         //Get Position of first quadcopter
	 *         if(!listPositions[0].empty())
	 *         {
	 *             for(int k = 0; k < 3; k++)
	 *             {
	 *                 first[k] = listPositions[0].back().getPosition()[k];
	 *             }
	 *         }
	 *         this->listPositionsMutex.unlock();
	 *         ROS_INFO("First set");
	 *         Position6DOF firstElement;
	 *         firstElement.setPosition(first);
	 *         /*this->listTargetsMutex.lock(); FIXME
	 *         this->listTargets[0].push_back(firstElement);
	 *         this->listTargetsMutex.unlock();*//*
	 *     }
	 *     else
	 *     {
	 *         //Set all the other positions according to the first crazyflie
	 *         ROS_INFO("Set the others");
	 *         target[0] += first[0];
	 *         target[1] += first[1];
	 *         target[2] += first[2];
	 *         Position6DOF targetElement;
	 *         targetElement.setPosition(target);
	 *         /*this->listTargetsMutex.lock(); FIXME
	 *         this->listTargets[i].push_back(targetElement);
	 *         this->listTargetsMutex.unlock();*//*
	 *         //If Shutdown has been called, abort.
	 *         shutdown = this->shutdownStarted;
	 *         if(shutdown)
	 *         {
	 *             return;
	 *         }
	 *         else
	 *         {
	 *             this->quadcopterMovementStatus[i] = CALCULATE_STABILIZE;
	 *             //this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
	 *         }
	 *     }
	 *     ROS_INFO("Inclining");
	 *     //Incline a little bit to avoid collisions (there is a level with the
	 * qc which are already in position and a moving level)
	 *     double pointer[3];
	 *     this->listTargetsMutex.lock();
	 *     for(int k = 0; k < 3; k++)
	 *     {
	 *         pointer[k] = this->listTargets[i].back().getPosition()[k];
	 *     }
	 *     this->listTargetsMutex.unlock();
	 *     //pointer[0] += 0;
	 *     //pointer[1] += 0;
	 *     pointer[2] += distance;
	 *     Position6DOF element;
	 *     element.setPosition(pointer);
	 *     /*this->listTargetsMutex.lock(); FIXME
	 *     this->listTargets[i].push_back(element);
	 *     this->listTargetsMutex.unlock();*//*
	 *     //If Shutdown has been called, abort.
	 *     shutdown = this->shutdownStarted;
	 *     if(shutdown)
	 *     {
	 *         return;
	 *     }
	 *     else
	 *     {
	 *         this->quadcopterMovementStatus[i] = CALCULATE_STABILIZE;
	 *         //this->quadcopterMovementStatus[i] = CALCULATE_MOVE;
	 *     }
	 *     ROS_INFO("Done with %i",i);
	 *     usleep(TIME_WAIT_AT_LANDING);
	 *    }*/
	ROS_INFO("CONTROLLER: BuildFormation finished");
	this->buildFormationFinished = true;
}

/**
 * Service to rotate formation
 */
bool Controller::rotateFormation(control_application::Rotation::Request  &req,
                                 control_application::Rotation::Response &res)
{
	// If a rotation is already running, return false and don't start a new one.
	if (this->rotationInProcess) {
		ROS_ERROR("CONTROLLER: Rotation is already in process. Please wait to start a new rotation");
		return false;
	}
	pthread_create(&tRotation, NULL, startThreadRotation, this);
	ROS_INFO("CONTROLLER: Thread tRotation set up");
	this->timeRotationStarted = getNanoTime();
	this->rotationInProcess = true;
	return true;
}

/**
 * Rotation of the formation around the center of the tracking Area or a certain
 * point.
 */
void Controller::rotate()
{
	int amount = this->formation->getAmount();
	long int currentTime = getNanoTime();
	// center of the tracking area around which we want to rotate
	Vector vector = this->trackingArea.getCenterOfTrackingArea();
	Position6DOF center = Position6DOF(vector);
	float rotationAngle = 2 * M_PI / amount;
	// Array which indicates which qc is already chosen for the rotation.
	bool qcChosen[amount];
	for (int i = 0; i < amount; i++) {
		qcChosen[i] = false;
	}
	// mapping betwenn quadcopter id and rotation position id.
	// qcPositionMap[rotation id] = qc id.
	int qcPositionMap[amount];
	Matrix2x2 rotationMatrix;
	for (int i = 0; i < amount; i++) {
		// Clear all previous Targets
		this->listTargetsMutex.lock();
		this->listTargets[i].clear();
		this->listTargetsMutex.unlock();
		Position6DOF newPosition;
		double *targetK = center.getPosition();
		// Calculate starting Positions for Rotation and the choose nearest qc
		// for that position
		if (i == 0) {
			targetK[0] += DISTANCE_ROTATE_TO_CENTER;
			qcPositionMap[0] = searchNeighbor(targetK, qcChosen);
			if (searchNeighbor(targetK, qcChosen) == -1) {
				ROS_DEBUG("CONTROLLER: No neighbor found");
			}
			qcChosen[qcPositionMap[0]] = true;
			newPosition.setPosition(targetK);
			this->listTargetsMutex.lock();
			this->listTargets[0].push_back(newPosition);
			this->listTargetsMutex.unlock();
		} else {
			// Rotate first point moved to z=0 level around origin, then move
			// back up
			Vector vectorK = Vector(DISTANCE_ROTATE_TO_CENTER, 0, 0);
			rotationMatrix =
			    Matrix2x2(cos(i * rotationAngle), -sin(i * rotationAngle), sin(i * rotationAngle), cos(
			                  i * rotationAngle));
			vectorK = rotationMatrix.multiplicate(vectorK);
			targetK[0] += vectorK.getV1();
			targetK[1] += vectorK.getV2();
			targetK[2] += vectorK.getV3();
			qcPositionMap[i] = searchNeighbor(targetK, qcChosen);
			if (searchNeighbor(targetK, qcChosen) == -1) {
				ROS_DEBUG("CONTROLLER: No neighbor found");
			}
			qcChosen[qcPositionMap[i]] = true;
			newPosition.setPosition(targetK);
			this->listTargetsMutex.lock();
			this->listTargets[i].push_back(newPosition);
			this->listTargetsMutex.unlock();
		}
	}
	// Start rotating. Set target for each qc to the previous target of the qc
	// next to the qc
	while (TIME_ROTATE_CIRCLE > (currentTime - this->timeRotationStarted)) {
		if (this->shutdownStarted) {
			return;
		}
		Position6DOF previousTarget;
		Position6DOF nextTarget;
		for (int i = 0; i < amount; i++) {
			int idCurrent = qcPositionMap[i];
			int idNext = qcPositionMap[i + 1];
			this->listTargetsMutex.lock();
			// Save first target because it gets overwritten
			if (i == 0) {
				previousTarget = this->listTargets[idNext].back();
			}
			if (i == amount - 1) {
				this->listTargets[idCurrent].push_back(previousTarget);
			} else {
				nextTarget = this->listTargets[idNext].back();
				this->listTargets[idCurrent].push_back(nextTarget);
			}
			this->listTargetsMutex.unlock();
		}
		currentTime = getNanoTime();
	}
	// Rotation finished
	this->rotationInProcess = false;
}

/**
 * Service to start build formation process
 */
bool Controller::startBuildFormation(control_application::BuildFormation::Request  &req,
                                     control_application::BuildFormation::Response &res)
{
	// If buildformation was already build, a new build request is ignored.
	if (this->buildFormationFinished) {
		ROS_ERROR("CONTROLLER: Formation was already build. No rebuildling allowed.");
		return false;
	}
	this->buildFormationStarted = true;
	pthread_create(&tBuildFormation, NULL, startThreadBuildFormation, this);
	ROS_INFO("CONTROLLER: Thread tBuildFormation set up");
	return true;
}

/**
 * Service to set Quadcopter IDs. Also initialize dynamic arrays according to
 * the given amount of qc and set starting target.
 * Also initialize multiple publisher
 */
bool Controller::setQuadcopters(control_application::SetQuadcopters::Request  &req,
                                control_application::SetQuadcopters::Response &res)
{
	// If setQuadcopters was already executed, a new setting request is ignored.
	if (this->receivedQuadcopters) {
		ROS_ERROR("CONTROLLER: Quadcopters already set. No resetting allowed");
		return false;
	}
	// If there are too less qcs to build the given formation, ignore request.
	if (this->receivedFormation && (req.amount < this->formation->getAmount())) {
		ROS_ERROR(
		    "CONTROLLER: You have too less quadcopters to create the formation. Please set more Quadcopters or restart the system.");
		return false;
	}
	ROS_INFO("CONTROLLER: Service setQuadcopters has been called amount %i", req.amount);
	ROS_DEBUG("CONTROLLER: Amount %i", req.amount);
	unsigned long int i;
	for (i = 0; i < req.amount; i++) {
		ROS_DEBUG("CONTROLLER: Quadcopter id: %i", req.quadcopterIds[i]);
		this->quadcopters.push_back(req.quadcopterIds[i]);
		this->quadcopterMovementStatus.push_back(CALCULATE_NONE);

		// Initialization of Arrays of Lists
		std::list<Position6DOF> newEmptyListPosition;
		this->listPositionsMutex.lock();
		this->listTargetsMutex.lock();
		this->listPositions.push_back(newEmptyListPosition);
		this->listTargets.push_back(newEmptyListPosition);
		if (this->receivedTrackingArea) {
			// Position6DOF defaultTarget =
			// Position6DOF(this->trackingArea.getCenterOfTrackingArea());
			Position6DOF defaultTarget = Position6DOF(-200, 1400, 300);
			// ROS_DEBUG("The target we want to set has z value: %f",
			// defaultTarget.getPosition()[2]);
			this->listTargets[i].push_back(defaultTarget);
			ROS_DEBUG("CONTROLLER: Set Target at Beginning is %f(z)", this->listTargets[i].back().getPosition()[2]);
			ROS_DEBUG("CONTROLLER: Set Target at Beginning is %f(y)", this->listTargets[i].back().getPosition()[1]);
			ROS_DEBUG("CONTROLLER: Set Target at Beginning is %f(x)", this->listTargets[i].back().getPosition()[0]);
		} else {
			Position6DOF defaultTarget = Position6DOF(0, 1400, 200);
			this->listTargets[i].push_back(defaultTarget);
			ROS_ERROR("CONTROLLER: Default target set");
		}
		this->receivedQuadStatus[i] = false; // received no quadcopter status
		                                     // information
		this->listTargetsMutex.unlock();
		this->listPositionsMutex.unlock();
		MovementQuadruple noMovement = MovementQuadruple(0, 0, 0, 0);
		std::list<MovementQuadruple> newEmptyListMovement;
		newEmptyListMovement.push_back(noMovement);
		this->listSentQuadruples.push_back(newEmptyListMovement);
		this->currentMovement.push_back(noMovement);
		ROS_INFO("CONTROLLER: Initialization done");

		// Subscriber to quadcopter status
		std::stringstream topicNameQS;
		int id = this->quadcopters[i];
		topicNameQS << "quadcopter_status_" << id;
		ROS_ERROR("CONTROLLER: Topic name: %s", topicNameQS.str().c_str());
		this->QuadStatus_sub[i] = this->n.subscribe<quadcopter_application::quadcopter_status>(
		    topicNameQS.str().c_str(), 1000, boost::bind(&Controller::quadStatusCallback, this, _1, id));

		// Publisher of Movement
		std::stringstream topicNameMov;
		topicNameMov << "quadcopter_movement_" << id;
		// Publisher for the Movement data of the Quadcopts (1000 is the max.
		// buffered messages)
		this->Movement_pub[i] = this->n.advertise<control_application::quadcopter_movement>(
		    topicNameMov.str().c_str(), 1000);

		// Publisher of is Tracked
		std::stringstream topicNameTrack;
		topicNameTrack << "quadcopter_is_tracked_" << id;
		// Publisher for the Movement data of the Quadcopts (1000 is the max.
		// buffered messages)
		this->Tracked_pub[i] = this->n.advertise<control_application::quadcopter_is_tracked>(
		    topicNameTrack.str().c_str(), 1000);
	}
	this->receivedQuadcopters = true;

	pthread_create(&tCalculateMovement, NULL, startThreadCalculateMovement, this);
	ROS_INFO("CONTROLLER: Thread tCalculateMovement set up");

	return true;
}

/**
 * Shutdown Service which starts a new thread for shutdownFormation
 */
bool Controller::shutdown(control_application::Shutdown::Request  &req, control_application::Shutdown::Response &res)
{
	if (this->shutdownStarted) {
		ROS_ERROR("CONTROLLER: Shutdown already started");
		return false;
	}
	ROS_INFO("CONTROLLER: Service shutdown has been called");
	pthread_create(&tShutdownFormation, NULL, &startThreadShutdown, this);
	ROS_INFO("CONTROLLER: Thread tShutdownFormation set up");
	return true;
}

/**
 * Shutdown Formation. Sets all Quadcopters on Land to start the landing
 * process. Then wait for them to all land.
 */
void Controller::shutdownFormation()
{
	if (this->shutdownStarted) {
		ROS_ERROR("CONTROLLER: Shutdown already started");
		return;
	}

	ROS_INFO("CONTROLLER: ShutdownFormation started");
	this->shutdownStarted = true; /* Start shutdown process */
	// int formationAmount = this->formation->getAmount();
	int formationAmount = 1;

	/* Bring all quadcopters to a hold */
	for (unsigned int i = 0; i < formationAmount; i++) {
		this->quadcopterMovementStatus[i] = CALCULATE_HOLD;
		this->emergencyShutdown[i] = false;
	}

	ROS_INFO("CONTROLLER: SHUTDOWN landFinished is %i", landFinished);
	while (!this->landFinished) {
		usleep(TIME_WAIT_FOR_LANDING);
	}
	ROS_INFO("CONTROLLER: Join threads");
	pthread_join(tCalculateMovement, 0);
	if (buildFormationStarted) {
		pthread_join(tBuildFormation, 0);
	}
	ROS_INFO("CONTROLLER: Shutdown function finished");
}

/**
 * Simple helper function to search for globalId in our mapping array and
 * returns the fitting localId
 */
int Controller::getLocalId(int globalId)    // TODO testme
{
	for (int i = 0; i < this->quadcopters.size(); i++) {
		if (globalId == this->quadcopters[i]) {
			return i;
		}
	}
	return INVALID;
}

/**
 * Checks if formation movement data and quadcopter positions have been received
 * lately and if the battery status is sufficient.
 * Otherwise calls emergencyroutine or simply send an error message.
 */
bool Controller::checkInput(int internId)
{
	// ROS_INFO("Checking");
	bool received = this->receivedQuadStatus[internId];
	unsigned int quadStatus = this->quadcopterMovementStatus[internId];
	/* Battery */
	if (this->battery_status[internId] < BATTERY_LOW && quadStatus != CALCULATE_NONE && received) {
		std::string message("Battery of Quadcopter %i is low (below %f). Shutdown formation\n", internId, BATTERY_LOW);
		// ROS_INFO("Battery of Quadcopter %i is low (below %f). Shutdown
		// formation\n", internId, BATTERY_LOW);
		// emergencyRoutine(message, internId);
	}
	long int currentTime = getNanoTime();
	long int lastForm = this->timeLastFormationMovement;
	if (currentTime - lastForm > ((long int) TIME_UPDATED_END) && quadStatus == CALCULATE_MOVE) {
		// std::string message = std::string("No new formation movement data has
		// been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		std::string message = "No new formation movement data has been received";
		// ROS_INFO("No new formation movement data has been received since %i
		// sec. Shutdown formation\n", TIME_UPDATED_END);
		// emergencyRoutine(message, internId);
		return false;
	}
	long int lastCur = this->timeLastCurrent[internId];
	if (currentTime - lastCur > ((long int) TIME_UPDATED_END) && quadStatus != CALCULATE_NONE && quadStatus !=
	    CALCULATE_START) {
		// ROS_DEBUG("Time difference %ld", currentTime - lastCur);
		// ROS_INFO("No quadcopter position data has been received since %i sec.
		// Shutdown formation\n", TIME_UPDATED_END);
		// std::string message2 = std::string("No quadcopter position data has
		// been received since %i sec. Shutdown formation\n", TIME_UPDATED_END);
		std::string message = "No new quadcopter position data has been received";
		ROS_ERROR("Left tracking area2");
		if (this->shutdownStarted == false) {
			emergencyShutdownRoutine(message);
		}

		// ROS_INFO("tracked false");
		tracked[internId] = false;
		control_application::quadcopter_is_tracked msg;
		msg.is_tracked = false;
		Tracked_pub[internId].publish(msg);
		return false;
	}
	if (currentTime - lastCur > TIME_UPDATED_CRITICAL && quadStatus != CALCULATE_NONE && quadStatus !=
	    CALCULATE_START) {
		std::string message = "No new quadcopter position data has been received";
		ROS_ERROR("CONTROLLER: Left trackin area");
		if (this->shutdownStarted == false) {
			this->emergencyShutdown[internId] = true;
			emergencyRoutine(message, internId);
		}
		tracked[internId] = false;
		control_application::quadcopter_is_tracked msg;
		msg.is_tracked = false;
		Tracked_pub[internId].publish(msg);
		return false;
	}

	return true;
}

/**
 * Emergency Routine. Gets started e.g. low battery status. Sends warning via
 * Ros and then starts the landing process for this qc.
 * If qc enters the tracking area again it will switch back to stabilizing.
 */
void Controller::emergencyRoutine(std::string message, int internId)
{
	ROS_INFO("CONTROLLER: Emergency Routine called");
	api_application::Message msg;
	msg.senderID = this->senderID;
	// Type 2 is a warning message
	msg.type = 2;
	msg.message = message;
	this->Message_pub.publish(msg);
	if (this->shutdownStarted == false) {
		this->quadcopterMovementStatus[internId] = CALCULATE_LAND;
	}
}

/**
 * Emergency Shutdown routine. Calls the Shutdown service to create a thread for
 * shutdownFormation.
 */
void Controller::emergencyShutdownRoutine(std::string message)
{
	ROS_INFO("CONTROLLER: Emergency Routine called");
	// Sends an error message to the api via ros
	api_application::Message msg;
	msg.senderID = this->senderID;
	// Type 2 is a warning message
	msg.type = 2;
	msg.message = message;
	this->Message_pub.publish(msg);
	if (!this->shutdownStarted) {
		ROS_INFO("CONTROLLER: I want to shutdown");
		control_application::Shutdown srv;
		if (Shutdown_client.call(srv)) {
			ROS_INFO("CONTROLLER: Shutdown call true");
		}
	}
}

/**
 * Callback for Ros Subscriber of Formation Movement
 */
void Controller::moveFormationCallback(const api_application::MoveFormation::ConstPtr &msg)
{
	if (this->rotationInProcess) {
		ROS_ERROR(
		    "CONTROLLER: Rotation in Process. Moving the formation is not allowed. Please wait for the rotation to be finished.");
		return;
	}
	ROS_INFO("CONTROLLER: I heard Movement. xMovement: %f", msg->xMovement);
	std::vector<float> movement;
	movement.push_back(msg->xMovement);
	movement.push_back(msg->yMovement);
	movement.push_back(msg->zMovement);
	this->formationMovementMutex.lock();
	this->formationMovement.push_back(movement);
	this->formationMovementMutex.unlock();
	this->timeLastFormationMovement = getNanoTime();

	// calculate and set a new target position each time there is new data
	if (this->buildFormationFinished) {
		setTargetPosition();
	}
}

/**
 * Callback for Ros Subscriber of set Formation
 */
void Controller::setFormationCallback(const api_application::SetFormation::ConstPtr &msg)
{
	if (this->receivedFormation) {
		ROS_ERROR("CONTROLLER: Formation was already set. No resetting allowed");
		return;
	}
	if (this->receivedQuadcopters && (this->quadcopterMovementStatus.size() < msg->amount)) {
		ROS_ERROR(
		    "CONTROLLER: Your formation has to many quadcopters set. Please set a different Formation or restart the system");
		return;
	}
	ROS_INFO("CONTROLLER: I heard Formation. amount: %i", msg->amount);
	this->formation->setDistance(msg->distance);
	this->formation->setAmount(msg->amount);
	// Iterate over all needed quadcopters for formation and set the formation
	// position of each quadcopter
	Position6DOF formPos[msg->amount];
	for (int i = 0; i < msg->amount; i++) {
		double pos[3];
		pos[0] = msg->xPositions[i];
		pos[1] = msg->yPositions[i];
		pos[2] = msg->zPositions[i];
		formPos[i].setPosition(pos);
	}
	this->formation->setFormationPosition(formPos);
	this->receivedFormation = true;
	ROS_INFO("CONTROLLER: Set Formation done");
	return;
}

/**
 * Callback for Ros Subscriber of quadcopter status
 */
void Controller::quadStatusCallback(const quadcopter_application::quadcopter_status::ConstPtr &msg, int topicNr)
{
	// ROS_INFO("I heard Quadcopter Status. topicNr: %i", topicNr);
	// Intern mapping
	int localQuadcopterId = this->getLocalId(topicNr);
	if (localQuadcopterId < 0) {
		// ROS_DEBUG("localQuadcopterId < 0!!!");
		return;
	}
	this->battery_status[localQuadcopterId] = msg->battery_status;
	this->roll_stab[localQuadcopterId] = msg->stabilizer_roll;
	this->pitch_stab[localQuadcopterId] = msg->stabilizer_pitch;
	this->yaw_stab[localQuadcopterId] = msg->stabilizer_yaw;
	// if(yaw_stab[localQuadcopterId] != 0)
	/*{
	 *     ROS_ERROR("Yaw is %f", this->yaw_stab[localQuadcopterId]);
	 *     ROS_ERROR("Yaw is %f", msg->stabilizer_yaw);
	 *    }*/
	this->thrust_stab[localQuadcopterId] = msg->stabilizer_thrust;
	this->baro[localQuadcopterId] = msg->baro;
	long int currentTime = getNanoTime();
	if (localQuadcopterId == 0 && currentTime > this->timeOffsetOutput + 1000000) {
		// ROS_INFO("bat: %f, roll: %f, pitch: %f, yaw: %f, thrust: %u",
		// msg->battery_status, msg->stabilizer_roll, msg->stabilizer_pitch,
		// msg->stabilizer_yaw, msg->stabilizer_thrust);
		this->timeOffsetOutput = currentTime;
	}
	if (!quadcopterStatus[localQuadcopterId].getQuadcopterThrust().initDone()) {
		/*
		 * Set spedific thrustvalues for each quadcopter once.
		 * Only if battery-value is useful.
		 */
		// ROS_ERROR("initDone is false");
		QuadcopterThrust qcThrust = this->quadcopterStatus[localQuadcopterId].getQuadcopterThrust();
		qcThrust.checkAndSetBatteryValue(this->battery_status[localQuadcopterId]);
		this->quadcopterStatus[localQuadcopterId].setQuadcopterThrust(qcThrust);
		// this->thrustHelp[localQuadcopterId] =
		// quadcopterStatus[localQuadcopterId].getQuadcopterThrust().getStart();

		this->baroTarget[localQuadcopterId] = this->baro[localQuadcopterId] + BARO_OFFSET;
		ROS_DEBUG("CONTROLLER: baroTarget set to %f", this->baroTarget[localQuadcopterId]);
	}
	this->receivedQuadStatus[localQuadcopterId] = true;

	this->batteryStatusCounter[localQuadcopterId]++;
	batteryStatusSum[localQuadcopterId] += this->battery_status[localQuadcopterId];
	int counterT = 5;
	if (this->batteryStatusCounter[localQuadcopterId] >= counterT) {
		batteryStatusSum[localQuadcopterId] = batteryStatusSum[localQuadcopterId] / counterT;
		QuadcopterThrust qcThrust = this->quadcopterStatus[localQuadcopterId].getQuadcopterThrust();
		qcThrust.setOffset(batteryStatusSum[localQuadcopterId]);
		this->quadcopterStatus[localQuadcopterId].setQuadcopterThrust(qcThrust);
		this->batteryStatusCounter[localQuadcopterId] = 0;
		this->batteryStatusSum[localQuadcopterId] = 0;
	}
}

/**
 * Callback for Ros Subscriber of system status. 1 = start, 2 = end
 */
void Controller::systemCallback(const api_application::System::ConstPtr &msg)
{
	ROS_INFO("CONTROLLER: I heard System. Status: %i", msg->command);
	if (msg->command == 1) {
		initialize();
	}
	if (msg->command == 2) {
		if (!this->shutdownStarted) {
			ROS_INFO("CONTROLLER: I want to shutdown");
			control_application::Shutdown srv;
			/*if(Shutdown_client.call(srv))
			 *    {
			 *     ROS_INFO("Shutdown call true");
			 *    }*/
			shutdown(srv.request, srv.response);
		}
	}
}

/**
 * Helper function for qc that are not supposed to move. Set all values to zero.
 */
void Controller::dontMove(int internId)
{
	MovementQuadruple newMovement = MovementQuadruple(0, 0, 0, 0);
	long int currentTime = getNanoTime();
	newMovement.setTimestamp(currentTime);
	this->currentMovement[internId] = newMovement;
}

/**
 * Helper function to increase thrust stepwise to start incremental.
 */
void Controller::moveUp(int internId)
{
	long int currentTime = getNanoTime();
	int thrustHelp = this->quadcopterStatus[internId].getQuadcopterThrust().getStart();
	MovementQuadruple newMovement = MovementQuadruple(thrustHelp, 0, 0, 0);
	newMovement.setTimestamp(currentTime);
	this->currentMovement[internId] = newMovement;
	int step = 200;
	// Increases thrust step by step to ensure slow inclining
	if ((currentTime > this->timeOffsetChangeThrust + 10000000) &&
	    (this->thrustHelp[internId] + step < this->quadcopterStatus[internId].getQuadcopterThrust().getStartMax())) {
		usleep(85000);
		this->thrustHelp[internId] += step;
		this->timeOffsetChangeThrust = getNanoTime();
	}
	// Protection mechanism for qc (either a too high thrust value or start
	// process took too long)
	if (thrustHelp >= quadcopterStatus[internId].getQuadcopterThrust().getStartMax() || currentTime >
	    this->timeDurationMoveup + 8000000000) {
		if (thrustHelp >= quadcopterStatus[internId].getQuadcopterThrust().getStartMax()) {
			ROS_DEBUG("CONTROLLER: Thrust too high");
		}
		if (currentTime > this->timeDurationMoveup + 8000000000) {
			ROS_DEBUG("Time over");
		}
		ROS_INFO("CONTROLLER: Emergency Shutdown Test");
		this->shutdownStarted = true;
		quadcopterMovementStatus[internId] = CALCULATE_LAND;
	}
}

/**
 * Helper function to stabilize qc with help of the pid controller. Calculates
 * different differences between x, y and z and controls qc.
 */
void Controller::stabilize(int internId)
{
	float rotationAngle = (this->yaw_stab[internId] / (float) 360) * (float) 2 * M_PI;
	// Matrix2x2 rotationMatrix = Matrix2x2(cos(rotationAngle),
	// -sin(rotationAngle), sin(rotationAngle), cos(rotationAngle));
	Matrix rotationMatrix = Matrix(cos(rotationAngle), -sin(rotationAngle), 0, sin(rotationAngle), cos(
	                                   rotationAngle), 0, 0, 0, 1);

	this->listPositionsMutex.lock();
	Position6DOF latestPosition = this->listPositions[internId].back();
	this->listPositionsMutex.unlock();
	double *position = latestPosition.getPosition();
	Vector vectorPos = Vector(position[0], position[1], position[2]);
	vectorPos = vectorPos.aftermult(rotationMatrix);
	latestPosition.setPosition(vectorPos);
	// ROS_INFO("Vector %f,%f,%f", vectorPos.getV1(), vectorPos.getV2(),
	// vectorPos.getV3());

	this->listTargetsMutex.lock();
	Position6DOF posTarget = this->listTargets[internId].back();
	this->listTargetsMutex.unlock();
	double *target = posTarget.getPosition();
	Vector vectorTarget = Vector(target[0], target[1], target[2]);
	vectorTarget = vectorTarget.aftermult(rotationMatrix);
	// vectorTarget.setV3(target[2]);
	posTarget.setPosition(vectorTarget);
	ROS_INFO("CONTROLLER: Yaw is: %f, Rotation %f, oldpos(%f,%f,%f) newpos(%f,%f,%f)", this->yaw_stab[internId],
	         rotationAngle,  position[0], position[1], position[2],
	         latestPosition.getPosition()[0], latestPosition.getPosition()[1], latestPosition.getPosition()[2]);

	MovementQuadruple newMovement = this->listSentQuadruples[internId].back();

	/* Thrust */
	double heightDiff = latestPosition.getDistanceZ(posTarget);
	// double baroDiff = baroTarget[internId] - baro[internId];
	// double calculatedThrust = controlThrust->getManipulatedVariable( baroDiff
	// );
	double calculatedThrust = controlThrust->getManipulatedVariable(heightDiff);
	// controlThrust->setOffset(
	// this->quadcopterStatus[internId].getQuadcopterThrust().getOffset() );
	unsigned int newThrust = quadcopterStatus[internId].getQuadcopterThrust().checkAndFix(calculatedThrust);
	newMovement.setThrust(newThrust);

	// MovementHelper helper;
	// Position6DOF posForRP = helper.prepareForRP(
	// quadcopterStatus[internId].getInfo().getRotation(), latestPosition,
	// posTarget );
	Position6DOF posForRP = latestPosition; // So far used without rotation

	/* Roll */
	float xDiff = posForRP.getDistanceX(posTarget);
	float newRoll = ((float) controlRoll->getManipulatedVariable(xDiff));

	/* Pitch */
	float yDiff = posForRP.getDistanceY(posTarget);
	float newPitch = ((float) controlPitch->getManipulatedVariable(yDiff));

	/* Yawrate */
	float yawDiff = 0.0 - this->yaw_stab[internId];
	float newYawrate = ((float) controlYawrate->getManipulatedVariable(yawDiff));

	/* Set values */
	ROS_INFO("  CONTROLLER: hDiff %f, calculated t %f, new %i", heightDiff, calculatedThrust, newThrust);
	// ROS_INFO("   baroDiff %f, new %i", baroDiff, newThrust);
	ROS_INFO("  CONTROLLER: xDiff %f, roll %f, yDiff %f, pitch %f", xDiff, newRoll, yDiff, newPitch);
	quadcopterStatus[internId].getInfo().checkAndFixRoll(newRoll);
	quadcopterStatus[internId].getInfo().checkAndFixPitch(newPitch);
	quadcopterStatus[internId].getInfo().checkAndFixYawrate(newYawrate);
	newMovement.setRollPitchYawrate(newRoll, newPitch, newYawrate);

	/* Set new Movement */
	this->currentMovement[internId] = newMovement;
}

/**
 * Helper function. No useful function right now. Might be used for reactive behavior
 * before landing.
 */
void Controller::hold(int internId)
{
	ROS_INFO("CONTROLLER: %i now land", internId);
	if (HOLD_SKIP) {
		quadcopterMovementStatus[internId] = CALCULATE_LAND;
		return;
	}
}

/**
 * Helper function to control the landing process. First decrease constantly
 * till qc left tracking area
 * and the decrease stepwise till zero.
 */
void Controller::land(int internId, int *nrLand)
{
	ROS_INFO("CONTROLLER: Land");
	long int currentTime = getNanoTime();
	// Decline until crazyflie isn't tracked anymore
	if (tracked[internId] == true) {
		ROS_INFO("CONTROLLER: Declining ros");
		MovementQuadruple newMovement = this->currentMovement[internId];
		if (currentMovement[internId].getThrust() < quadcopterStatus[internId].getQuadcopterThrust().getDecline()) {
			newMovement.setThrust(currentMovement[internId].getThrust());
		} else {
			newMovement.setThrust(quadcopterStatus[internId].getQuadcopterThrust().getDecline());
		}
		newMovement.setTimestamp(currentTime);
		this->currentMovement[internId] = newMovement;
		this->timeOffsetChangeThrust = getNanoTime();
	} else {
		if (this->thrustHelp[internId] > quadcopterStatus[internId].getQuadcopterThrust().getDecline()) { //
		                                                                                                  // FIXME
			if (currentMovement[internId].getThrust() < quadcopterStatus[internId].getQuadcopterThrust().getDecline()) {
				this->thrustHelp[internId] = currentMovement[internId].getThrust();
			} else {
				ROS_DEBUG("CONTROLLER: Decline newly set in land");
				this->thrustHelp[internId] = quadcopterStatus[internId].getQuadcopterThrust().getDecline(); //
				                                                                                            // FIXME
			}
			this->timeOffsetChangeThrust = getNanoTime();
		}
		int step = 0;
		if (this->shutdownStarted) {
			step = DECLINE_SHUTDOWN_STEP;
		} else {
			step = DECLINE_NOT_TRACKED_STEP;
		}
		if (currentTime > this->timeOffsetChangeThrust + 1000000 && this->thrustHelp[internId] - step > 0) {
			ROS_DEBUG("CONTROLLER: Lower Thrust in land");
			usleep(85000); // FIXME sleep here? still needed?
			this->thrustHelp[internId] -= step;
			this->timeOffsetChangeThrust = getNanoTime();
		}
		// Shutdown crazyflie after having left the tracking area.
		MovementQuadruple newMovement = MovementQuadruple(this->thrustHelp[internId], 0, 0, 0);
		newMovement.setTimestamp(currentTime);
		ROS_DEBUG("CONTROLLER: LAND THRUST %i", newMovement.getThrust());
		this->currentMovement[internId] = newMovement;
		if (this->thrustHelp[internId] - step <= 0) {
			this->quadcopterMovementStatus[internId] = CALCULATE_NONE;
			dontMove(internId);
			(*nrLand)++;
			ROS_INFO("CONTROLLER: Landed: %i", *nrLand);
		}
	}
}

/**
 * Helper function to check if two positions are close/ in a specific range to
 * each other.
 */
static bool closeToTarget(Position6DOF position1, Position6DOF position2, double range)
{
	double distance = position1.getAbsoluteDistance(position2);
	if (distance < range) {
		return true;
	}
	return false;
}

/**
 * Helper function to search for a close neighbor of target quadcopter and
 * return the wanted id.
 */
int Controller::searchNeighbor(double *target, bool *ids)
{
	float distance = -1;
	int neighborId = -1;
	for (int i = 0; i < this->formation->getAmount(); i++) {
		if (ids[i]) {
			continue;
		}
		this->listPositionsMutex.lock();
		Position6DOF currentPosition = this->listPositions[i].back();
		this->listPositionsMutex.unlock();
		double *currentPos = currentPosition.getPosition();
		float dX = abs(currentPos[0] - target[0]);
		float dY = abs(currentPos[1] - target[1]);
		float dZ = abs(currentPos[2] - target[2]);
		float distanceHelp = sqrt(dX * dX + dY * dY + dZ * dZ);
		if ((distance == -1) || (distance > distanceHelp)) {
			neighborId = i;
			distance = distanceHelp;
		}
	}
	return neighborId;
}

/**
 * Helper function to start a new thread for movement calculation
 */
void* startThreadCalculateMovement(void *something)
{
	Controller *someOther = (Controller*) something;
	someOther->calculateMovement();
}

/**
 * Helper function to start a new thread for formation building
 */
void* startThreadBuildFormation(void *something)
{
	Controller *someOther = (Controller*) something;
	someOther->buildFormation();
}

/**
 * Helper function to start a new thread for formation shutdown
 */
void* startThreadShutdown(void *something)
{
	Controller *someOther = (Controller*) something;
	someOther->shutdownFormation();
}

/**
 * Helper function to start a new thread for formation rotation
 */
void* startThreadRotation(void *something)
{
	Controller *someOther = (Controller*) something;
	someOther->rotate();
}
