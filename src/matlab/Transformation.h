#ifndef TRANSFORMATION_H
#define FORMATION_HPP

class Transformation {
public:
	Transformation();
private:
	double rotationY[3][3];
	double rotationZ[3][3];
	
	double matrixMGlobal[3][3]; // M Schlange
	int rotateSpecific;	// 0, 120 or -120
	double matrixMSpecific[3][3]; 	// M klein-A,B oder C
	
	double translation[3];
};


#endif // TRANSFORMATION_H
