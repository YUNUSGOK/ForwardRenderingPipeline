#include <iostream>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <cmath>

#include "Scene.h"
#include "Camera.h"
#include "Color.h"
#include "Model.h"
#include "Rotation.h"
#include "Scaling.h"
#include "Translation.h"
#include "Triangle.h"
#include "Vec3.h"
#include "tinyxml2.h"
#include "Helpers.h"

using namespace tinyxml2;
using namespace std;

/*
	Transformations, clipping, culling, rasterization are done here.
	You can define helper functions inside Scene class implementation.
*/

Matrix4 Scene::getMcam(Camera *camera) {
	Vec3 u = camera->u;
	Vec3 v = camera->v;
	Vec3 w = camera->w;

	Vec3 e = camera->pos;

	double R[4][4] {
		{u.x, u.y, u.z, 0},
		{v.x, v.y, v.z, 0},
		{w.x, w.y, w.z, 0},
		{0, 0, 0, 1},
	};
	double T[4][4]={
		{1, 0, 0, -e.x},
		{0, 1, 0, -e.y},
		{0, 0, 1, -e.z},
		{0, 0, 0, 1}
	};

	return multiplyMatrixWithMatrix(Matrix4(R), Matrix4(T));

}

Matrix4 Scene::getMp2o(Camera *camera)
{
	double n = camera->near;
	double f = camera->far;
	double m[4][4] = {
							{n, 0, 0, 0},
							{0, n, 0, 0},
							{0, 0, f+n, f*n},
							{0, 0, -1, 0}
						};
	return Matrix4(m);
}



Matrix4 Scene::getMorth(Camera *camera){
	double n = camera->near;
	double f = camera->far;
	double r = camera->right;
	double l = camera->left;
	double t = camera->top;
	double b = camera->bottom;
	double m[4][4]=
	{
		{2/(r-l),0,0, -(r+l)/(r-l)},
		{0,2/(t-b),0, -(t+b)/(t-b)},
		{0,0,-2/(f-n), -(f+n)/(f-n)},
		{0,0,0,1}
	};
	return Matrix4(m);

}
Matrix4 Scene::getMvp(Camera *camera){
	double m[4][4]=
	{
		{(float)(camera->horRes)/2,0,0,((float)(camera->horRes-1))/2	},
		{0,(float)(camera->verRes)/2,0,(float)(camera->verRes-1)/2	},
		{0,0,0.5,0.5},
		{0,0,0,0}
	};
	return Matrix4(m);
}


Matrix4 Scene::getTransMatrix(Translation *trans)
{
	double t[4][4] = {
						{1, 0, 0, trans->tx},
						{0, 1, 0, trans->ty},
						{0, 0, 1, trans->tz},
						{0, 0, 0, 1}
					};

	return Matrix4(t);
}

Matrix4 Scene::getScalingMatrix(Scaling* scale)
{
	double s[4][4] = {
						{scale->sx, 0, 0, 0},
						{0, scale->sy, 0, 0},
						{0, 0, scale->sz, 0},
						{0, 0, 0, 1}
					};
	return Matrix4(s);

}


Matrix4 Scene::getRotMatrix(Rotation *rot)
{
	Vec3 u(rot->ux, rot->uy, rot->uz, -1);
	Vec3 v(-rot->uy, rot->ux, 0, -1);
	Vec3 w(crossProductVec3(u, v));
	v = normalizeVec3(v);
	w = normalizeVec3(w);
	double angle = rot->angle;
	double r[4][4];
	double M[4][4] = {
						{u.x, u.y, u.z, 0},
						{v.x, v.y, v.z, 0},
						{w.x, w.y, w.z, 0},
						{0, 0, 0, 1}
					};
	double inverseM[4][4] =  {
								{u.x, v.x, w.x, 0},
								{u.y, v.y, w.y, 0},
								{u.z, v.z, w.z, 0},
								{0, 0, 0, 1}
							};
	double Ra[4][4] = {
						{1, 0, 0, 0},
						{0, cos(angle*M_PI/180.0), -sin(angle*M_PI/180.0) , 0},
						{0, sin(angle*M_PI/180.0), cos(angle*M_PI/180.0), 0},
						{0, 0, 0, 1}
					};

	Matrix4 RaM(multiplyMatrixWithMatrix(Matrix4(Ra), Matrix4(M)));

	return multiplyMatrixWithMatrix(Matrix4(inverseM), RaM);
}

Matrix4 Scene::getMmodel(Model * model)
{
	Matrix4 Mmodel(getIdentityMatrix());
	for(int i=0; i<model->numberOfTransformations ;i++)
	{
		if(model->transformationTypes[i]=='r')
			Mmodel = multiplyMatrixWithMatrix(getRotMatrix(rotations[model->transformationIds[i]-1]),Mmodel);

		else if(model->transformationTypes[i]=='s')
			Mmodel = multiplyMatrixWithMatrix(getScalingMatrix(scalings[model->transformationIds[i]-1]),Mmodel);

		else if(model->transformationTypes[i]=='t')
			Mmodel = multiplyMatrixWithMatrix(getTransMatrix(translations[model->transformationIds[i]-1]),Mmodel);
	}
	return Mmodel;
}


void Scene::midpoint1(Vec4 &v1, Vec4 &v2 )
{
	int y = v1.y;
	int dx = v2.x-v1.x ;
	int dy = (v1.y-v2.y);
	int d = 2*dy + dx;
	Color c1 = *colorsOfVertices[v1.colorId-1];
	Color c2 = *colorsOfVertices[v2.colorId-1];
	Color c = c1;
	Color deltaC = (c2-c1)/dx;


	for(int x=v1.x; x<=v2.x ; x++ )
	{
		image[x][y] = c.clippedColor();
		if(d<0)
		{
			y++;
			d += 2*(v1.y-v2.y) + 2*( v2.x-v1.x);
		}
		else
		{
			d +=2*(v1.y-v2.y);
		}
		c = c+ deltaC;
	}
}


void Scene::midpoint2(Vec4 &v1, Vec4 &v2 )
{

	int x = v1.x;
	int dx = v1.x-v2.x ;
	int dy = (v2.y-v1.y);
	int d = 2*dx + dy;
	Color c1 = *colorsOfVertices[v1.colorId-1];
	Color c2 = *colorsOfVertices[v2.colorId-1];
	Color c = c1;
	Color deltaC = (c2-c1)/dy;


	for(int y=v1.y; y<=v2.y ; y++ )
	{
		image[x][y] = c.clippedColor();
		if(d<0)
		{
			x++;
			d += 2*(v1.x-v2.x) + 2*( v2.y-v1.y);
		}
		else
		{
			d +=2*(v1.x-v2.x);
		}
		c= c+ deltaC;
	}

}



void Scene::midpoint3(Vec4 &v1, Vec4 &v2 )
{

	int y = v2.y;
	int dx = v1.x-v2.x ;
	int dy = v2.y-v1.y ;
	int d = 2*dy -dx;

	Color c1 = *colorsOfVertices[v1.colorId-1];
	Color c2 = *colorsOfVertices[v2.colorId-1];
	Color c = c2;
	Color deltaC = (c1-c2)/(dx);

	for(int x=v2.x; x<v1.x ; x++ )
	{
		image[x][y] = c.clippedColor();
		if(d<=0)
		{

			d +=2*dy;
		}
		else
		{
			d += 2*(dy-dx);
			y-=1;
		}
		c = c+deltaC;
	}

}

void Scene::midpoint4(Vec4 &v1, Vec4 &v2 )
{
	int x = v1.x;
	int dx = v1.x-v2.x ;
	int dy = v2.y-v1.y ;
	int d = 2*dx -dy;

	Color c1 = *colorsOfVertices[v1.colorId-1];
	Color c2 = *colorsOfVertices[v2.colorId-1];
	Color c = c1;
	Color deltaC = (c2-c1)/(dy);

	for(int y=v1.y; y<v2.y ; y++ )
	{
		image[x][y] = c.clippedColor();
		if(d<=0)
		{

			d +=2*dx;
		}
		else
		{
			d += 2*(dx-dy);
			x-=1;
		}
		c= c+deltaC;
	}

}



void Scene::rasterline(Vec4 &v1, Vec4 &v2 )
{
	if(slope(v1,v2)<=1 && slope(v1,v2)>=0)
	{
		if(v1.x< v2.x)
		midpoint1(v1,v2);
		else midpoint1(v2,v1);
	}

	if( slope(v1,v2)>1)
	{
		if(v1.x<v2.x)
		midpoint2(v1,v2);
		else
		midpoint2(v2,v1);
	}
	if( slope(v1,v2)>-1 && slope(v1,v2)<0)
	{
		if(v1.x>v2.x)
		midpoint3(v1,v2);
		else
		midpoint3(v2,v1);
	}
	if( slope(v1,v2)<-1)
	{
		if(v2.y > v1.y)
		midpoint4(v1,v2);
		else
		midpoint4(v2,v1);
	}


}


void Scene::rastertriangle(Vec4 &v0,Vec4 &v1,Vec4 &v2 )
{
	double a,b,c;
	double ymin = threemin(v0.y, v1.y, v2.y);
	double ymax = threemax(v0.y, v1.y, v2.y);
	double xmin = threemin(v0.x, v1.x, v2.x);
	double xmax = threemax(v0.x, v1.x, v2.x);

	Color c0( *colorsOfVertices[v0.colorId-1]);
	Color c1( *colorsOfVertices[v1.colorId-1]);
	Color c2( *colorsOfVertices[v2.colorId-1]);
	Color cres;

	for(int i=xmin; i<xmax; i++)
	{
		for(int j = ymin ; j<ymax ; j++)
		{
			a = lineEq(i,j,v1,v2)/lineEq(v0.x,v0.y,v1,v2);
			b = lineEq(i,j,v2,v0)/lineEq(v1.x,v1.y,v2,v0);
			c = lineEq(i,j,v0,v1)/lineEq(v2.x,v2.y,v0,v1);
			if(a>=0 &&b>=0 &&c>=0 )
			{

				cres = c0*a+c1*b+c2*c;
				image[i][j] = cres.clippedColor();
			}
		}
	}


}

bool Scene::backFaceCulling(Vec4 &v0, Vec4 &v1,Vec4 &v2,Vec3 e)
{
	Vec3 p0(v0.x,v0.y,v0.z,v0.colorId);
	Vec3 p1(v1.x,v1.y,v1.z,v1.colorId);
	Vec3 p2(v2.x,v2.y,v2.z,v2.colorId);

	Vec3 n = crossProductVec3(subtractVec3(p2,p1),subtractVec3(p0,p1));
	return 0 > dotProductVec3(n,subtractVec3(p1,e));
}

void Scene::forwardRenderingPipeline(Camera *camera)
{


		int modelSize =models.size();
		int triSize;
		Model *model;
		Matrix4 Mtotal(getIdentityMatrix());

		Matrix4 Mcam(getMcam(camera));
		Matrix4 Mvp(getMvp(camera));
		Matrix4 MOrth(getMorth(camera));
		Matrix4 Mp2o(getIdentityMatrix());

		if(projectionType==1){
			Mp2o=multiplyMatrixWithMatrix(Mp2o,getMp2o(camera));
		}
;


		for(int modelNum=0; modelNum<modelSize; modelNum++)
		{
			model = models[modelNum];

			Mtotal = multiplyMatrixWithMatrix(getMmodel(model),Mtotal);

			Mtotal = multiplyMatrixWithMatrix(Mcam,Mtotal);

			Mtotal = multiplyMatrixWithMatrix(Mp2o,Mtotal);

			Mtotal = multiplyMatrixWithMatrix(MOrth,Mtotal);
			// std::cout << Mtotal << '\n';
			triSize = model->numberOfTriangles;
			for(int triNum=0; triNum<triSize ; triNum++ )
			{
				Triangle tri = model->triangles[triNum];

				Vec3 *vertex1  = vertices[tri.vertexIds[0]-1];
				Vec3 *vertex2  = vertices[tri.vertexIds[1]-1];
				Vec3 *vertex3  = vertices[tri.vertexIds[2]-1];
				Vec4 v1(vertex1->x, vertex1->y, vertex1->z, 1, vertex1->colorId);

				Vec4 v2(vertex2->x, vertex2->y, vertex2->z, 1, vertex2->colorId);

				Vec4 v3(vertex3->x, vertex3->y, vertex3->z, 1, vertex3->colorId);



				v1= multiplyMatrixWithVec4(Mtotal,v1);
				v2= multiplyMatrixWithVec4(Mtotal,v2);
				v3= multiplyMatrixWithVec4(Mtotal,v3);

				//culling
				//clipping

				//start::perspective divide
				v1 = multiplyVec4WithScalar(v1,1/v1.t);
				v2 = multiplyVec4WithScalar(v2,1/v2.t);
				v3 = multiplyVec4WithScalar(v3,1/v3.t);
				//end::perspective divide

				//start::ViewPort Transformation

				v1 = multiplyMatrixWithVec4(Mvp,v1);
				v2 = multiplyMatrixWithVec4(Mvp,v2);
				v3 = multiplyMatrixWithVec4(Mvp,v3);

				//end::ViewPort Transformation

				if(cullingEnabled ==1)
				{
					if(backFaceCulling(v1,v2,v3,camera->pos))
					continue;
				}
				if(model->type ==0)//wireframe
				{
					rasterline(v1,v2);
					rasterline(v2,v3);
					rasterline(v3,v1);
				}
				if(model->type ==1)//solid
				{

					rastertriangle(v1,v2,v3);

				}

				/*




				*/
			}


		}



}

/*
	Parses XML file
*/
Scene::Scene(const char *xmlPath)
{
	const char *str;
	XMLDocument xmlDoc;
	XMLElement *pElement;

	xmlDoc.LoadFile(xmlPath);

	XMLNode *pRoot = xmlDoc.FirstChild();

	// read background color
	pElement = pRoot->FirstChildElement("BackgroundColor");
	str = pElement->GetText();
	sscanf(str, "%lf %lf %lf", &backgroundColor.r, &backgroundColor.g, &backgroundColor.b);

	// read culling
	pElement = pRoot->FirstChildElement("Culling");
	if (pElement != NULL)
		pElement->QueryBoolText(&cullingEnabled);

	// read projection type
	pElement = pRoot->FirstChildElement("ProjectionType");
	if (pElement != NULL)
		pElement->QueryIntText(&projectionType);

	// read cameras
	pElement = pRoot->FirstChildElement("Cameras");
	XMLElement *pCamera = pElement->FirstChildElement("Camera");
	XMLElement *camElement;
	while (pCamera != NULL)
	{
		Camera *cam = new Camera();

		pCamera->QueryIntAttribute("id", &cam->cameraId);

		camElement = pCamera->FirstChildElement("Position");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->pos.x, &cam->pos.y, &cam->pos.z);

		camElement = pCamera->FirstChildElement("Gaze");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->gaze.x, &cam->gaze.y, &cam->gaze.z);

		camElement = pCamera->FirstChildElement("Up");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf", &cam->v.x, &cam->v.y, &cam->v.z);

		cam->gaze = normalizeVec3(cam->gaze);
		cam->u = crossProductVec3(cam->gaze, cam->v);
		cam->u = normalizeVec3(cam->u);

		cam->w = inverseVec3(cam->gaze);
		cam->v = crossProductVec3(cam->u, cam->gaze);
		cam->v = normalizeVec3(cam->v);

		camElement = pCamera->FirstChildElement("ImagePlane");
		str = camElement->GetText();
		sscanf(str, "%lf %lf %lf %lf %lf %lf %d %d",
			   &cam->left, &cam->right, &cam->bottom, &cam->top,
			   &cam->near, &cam->far, &cam->horRes, &cam->verRes);

		camElement = pCamera->FirstChildElement("OutputName");
		str = camElement->GetText();
		cam->outputFileName = string(str);

		cameras.push_back(cam);

		pCamera = pCamera->NextSiblingElement("Camera");
	}

	// read vertices
	pElement = pRoot->FirstChildElement("Vertices");
	XMLElement *pVertex = pElement->FirstChildElement("Vertex");
	int vertexId = 1;

	while (pVertex != NULL)
	{
		Vec3 *vertex = new Vec3();
		Color *color = new Color();

		vertex->colorId = vertexId;

		str = pVertex->Attribute("position");
		sscanf(str, "%lf %lf %lf", &vertex->x, &vertex->y, &vertex->z);

		str = pVertex->Attribute("color");
		sscanf(str, "%lf %lf %lf", &color->r, &color->g, &color->b);

		vertices.push_back(vertex);
		colorsOfVertices.push_back(color);

		pVertex = pVertex->NextSiblingElement("Vertex");

		vertexId++;
	}

	// read translations
	pElement = pRoot->FirstChildElement("Translations");
	XMLElement *pTranslation = pElement->FirstChildElement("Translation");
	while (pTranslation != NULL)
	{
		Translation *translation = new Translation();

		pTranslation->QueryIntAttribute("id", &translation->translationId);

		str = pTranslation->Attribute("value");
		sscanf(str, "%lf %lf %lf", &translation->tx, &translation->ty, &translation->tz);

		translations.push_back(translation);

		pTranslation = pTranslation->NextSiblingElement("Translation");
	}

	// read scalings
	pElement = pRoot->FirstChildElement("Scalings");
	XMLElement *pScaling = pElement->FirstChildElement("Scaling");
	while (pScaling != NULL)
	{
		Scaling *scaling = new Scaling();

		pScaling->QueryIntAttribute("id", &scaling->scalingId);
		str = pScaling->Attribute("value");
		sscanf(str, "%lf %lf %lf", &scaling->sx, &scaling->sy, &scaling->sz);

		scalings.push_back(scaling);

		pScaling = pScaling->NextSiblingElement("Scaling");
	}

	// read rotations
	pElement = pRoot->FirstChildElement("Rotations");
	XMLElement *pRotation = pElement->FirstChildElement("Rotation");
	while (pRotation != NULL)
	{
		Rotation *rotation = new Rotation();

		pRotation->QueryIntAttribute("id", &rotation->rotationId);
		str = pRotation->Attribute("value");
		sscanf(str, "%lf %lf %lf %lf", &rotation->angle, &rotation->ux, &rotation->uy, &rotation->uz);

		rotations.push_back(rotation);

		pRotation = pRotation->NextSiblingElement("Rotation");
	}

	// read models
	pElement = pRoot->FirstChildElement("Models");

	XMLElement *pModel = pElement->FirstChildElement("Model");
	XMLElement *modelElement;
	while (pModel != NULL)
	{
		Model *model = new Model();

		pModel->QueryIntAttribute("id", &model->modelId);
		pModel->QueryIntAttribute("type", &model->type);

		// read model transformations
		XMLElement *pTransformations = pModel->FirstChildElement("Transformations");
		XMLElement *pTransformation = pTransformations->FirstChildElement("Transformation");

		pTransformations->QueryIntAttribute("count", &model->numberOfTransformations);

		while (pTransformation != NULL)
		{
			char transformationType;
			int transformationId;

			str = pTransformation->GetText();
			sscanf(str, "%c %d", &transformationType, &transformationId);

			model->transformationTypes.push_back(transformationType);
			model->transformationIds.push_back(transformationId);

			pTransformation = pTransformation->NextSiblingElement("Transformation");
		}

		// read model triangles
		XMLElement *pTriangles = pModel->FirstChildElement("Triangles");
		XMLElement *pTriangle = pTriangles->FirstChildElement("Triangle");

		pTriangles->QueryIntAttribute("count", &model->numberOfTriangles);

		while (pTriangle != NULL)
		{
			int v1, v2, v3;

			str = pTriangle->GetText();
			sscanf(str, "%d %d %d", &v1, &v2, &v3);

			model->triangles.push_back(Triangle(v1, v2, v3));

			pTriangle = pTriangle->NextSiblingElement("Triangle");
		}

		models.push_back(model);

		pModel = pModel->NextSiblingElement("Model");
	}
}

/*
	Initializes image with background color
*/
void Scene::initializeImage(Camera *camera)
{
	if (this->image.empty())
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			vector<Color> rowOfColors;

			for (int j = 0; j < camera->verRes; j++)
			{
				rowOfColors.push_back(this->backgroundColor);
			}

			this->image.push_back(rowOfColors);
		}
	}
	// if image is filled before, just change color rgb values with the background color
	else
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			for (int j = 0; j < camera->verRes; j++)
			{
				this->image[i][j].r = this->backgroundColor.r;
				this->image[i][j].g = this->backgroundColor.g;
				this->image[i][j].b = this->backgroundColor.b;
			}
		}
	}
}

/*
	If given value is less than 0, converts value to 0.
	If given value is more than 255, converts value to 255.
	Otherwise returns value itself.
*/
int Scene::makeBetweenZeroAnd255(double value)
{
	if (value >= 255.0)
		return 255;
	if (value <= 0.0)
		return 0;
	return (int)(value);
}

/*
	Writes contents of image (Color**) into a PPM file.
*/
void Scene::writeImageToPPMFile(Camera *camera)
{
	ofstream fout;

	fout.open(camera->outputFileName.c_str());

	fout << "P3" << endl;
	fout << "# " << camera->outputFileName << endl;
	fout << camera->horRes << " " << camera->verRes << endl;
	fout << "255" << endl;

	for (int j = camera->verRes - 1; j >= 0; j--)
	{
		for (int i = 0; i < camera->horRes; i++)
		{
			fout << makeBetweenZeroAnd255(this->image[i][j].r) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].g) << " "
				 << makeBetweenZeroAnd255(this->image[i][j].b) << " ";
		}
		fout << endl;
	}
	fout.close();
}

/*
	Converts PPM image in given path to PNG file, by calling ImageMagick's 'convert' command.
	os_type == 1 		-> Ubuntu
	os_type == 2 		-> Windows
	os_type == other	-> No conversion
*/
void Scene::convertPPMToPNG(string ppmFileName, int osType)
{
	string command;

	// call command on Ubuntu
	if (osType == 1)
	{
		command = "convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// call command on Windows
	else if (osType == 2)
	{
		command = "magick convert " + ppmFileName + " " + ppmFileName + ".png";
		system(command.c_str());
	}

	// default action - don't do conversion
	else
	{
	}
}
