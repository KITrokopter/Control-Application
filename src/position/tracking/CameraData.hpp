#pragma once

typedef struct {
	Vector cameraVector;
	int camNo;
	int quadcopterId;
	long int time;
	bool valid;
} CameraData;