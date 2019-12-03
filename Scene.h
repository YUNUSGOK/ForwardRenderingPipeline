#ifndef _SCENE_H_
#define _SCENE_H_

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

#include "Matrix4.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "Vec4.h"

using namespace std;

class Scene
{
public:
	Color backgroundColor;
	bool cullingEnabled;
	int projectionType;

	vector< vector<Color> > image;
	vector< Camera* > cameras;
	vector< Vec3* > vertices;
	vector< Color* > colorsOfVertices;
	vector< Scaling* > scalings;
	vector< Rotation* > rotations;
	vector< Translation* > translations;
	vector< Model* > models;

	Scene(const char *xmlPath);

	void initializeImage(Camera* camera);
	void forwardRenderingPipeline(Camera* camera);
	int makeBetweenZeroAnd255(double value);
	void writeImageToPPMFile(Camera* camera);
	void convertPPMToPNG(string ppmFileName, int osType);

	Matrix4 getMmodel(Model * model);
	Matrix4 getMcam(Camera *camera);
	Matrix4 getMp2o(Camera *camera);
	Matrix4 getMorth(Camera *camera);
	Matrix4 getMvp(Camera *camera);
	Matrix4 getTransMatrix(Translation *trans);
	Matrix4 getScalingMatrix(Scaling *scale);
	Matrix4 getRotMatrix(Rotation *rot);
	void midpoint1(Vec4 &v1, Vec4 &v2 );
	void midpoint2(Vec4 &v1, Vec4 &v2 );
	void midpoint3(Vec4 &v1, Vec4 &v2 );
	void midpoint4(Vec4 &v1, Vec4 &v2 );
	void rasterline(Vec4 &v1, Vec4 &v2 );
	void rastertriangle(Vec4 &v0,Vec4 &v1,Vec4 &v2 );
	bool backFaceCulling(Vec4 &v1, Vec4 &v2,Vec4 &v3,Vec3 e);
};

#endif
