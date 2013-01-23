#include "ofxBulletConvexDecomposer.h"

#include "hacdCircularList.h"
#include "hacdVector.h"
#include "hacdICHull.h"
#include "hacdGraph.h"
#include "hacdHACD.h"

//#include "cd_wavefront.h"
#include "ConvexBuilder.h"

#include "btBulletDynamicsCommon.h"

#include "ofxBulletCustomShape.h"


pair<btVector3,btConvexHullShape*> createConvexHullShapeFromConvexResult(ConvexResult &result, btVector3 localScaling )
{
	
	btTriangleMesh* trimesh = new btTriangleMesh();
	//m_convexDemo->m_trimeshes.push_back(trimesh);
	btConvexHullShape* convexShape = NULL;
	btVector3 centroid(0,0,0);
	
	//calc centroid, to shift vertices around center of mass
	
	btAlignedObjectArray<btVector3> vertices;
	if ( 1 )
	{
		//const unsigned int *src = result.mHullIndices;
		for (unsigned int i=0; i<result.mHullVcount; i++)
		{
			btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);
			vertex *= localScaling;
			centroid += vertex;
			
		}
	}
	
	centroid *= 1.f/(float(result.mHullVcount) );
	
	if ( 1 )
	{
		//const unsigned int *src = result.mHullIndices;
		for (unsigned int i=0; i<result.mHullVcount; i++)
		{
			btVector3 vertex(result.mHullVertices[i*3],result.mHullVertices[i*3+1],result.mHullVertices[i*3+2]);
			vertex *= localScaling;
			vertex -= centroid ;
			vertices.push_back(vertex);
		}
	}
	
	
	
	if ( 1 )
	{
		const unsigned int *src = result.mHullIndices;
		for (unsigned int i=0; i<result.mHullTcount; i++)
		{
			unsigned int index0 = *src++;
			unsigned int index1 = *src++;
			unsigned int index2 = *src++;
			
			
			btVector3 vertex0(result.mHullVertices[index0*3], result.mHullVertices[index0*3+1],result.mHullVertices[index0*3+2]);
			btVector3 vertex1(result.mHullVertices[index1*3], result.mHullVertices[index1*3+1],result.mHullVertices[index1*3+2]);
			btVector3 vertex2(result.mHullVertices[index2*3], result.mHullVertices[index2*3+1],result.mHullVertices[index2*3+2]);
			vertex0 *= localScaling;
			vertex1 *= localScaling;
			vertex2 *= localScaling;
			
			vertex0 -= centroid;
			vertex1 -= centroid;
			vertex2 -= centroid;
			
			trimesh->addTriangle(vertex0,vertex1,vertex2);
			
		}
	}
	
	//	float mass = 1.f;
	
	
	//this is a tools issue: due to collision margin, convex objects overlap, compensate for it here:
	//#define SHRINK_OBJECT_INWARDS 1
#ifdef SHRINK_OBJECT_INWARDS
	
	float collisionMargin = 0.01f;
	
	btAlignedObjectArray<btVector3> planeEquations;
	btGeometryUtil::getPlaneEquationsFromVertices(vertices,planeEquations);
	
	btAlignedObjectArray<btVector3> shiftedPlaneEquations;
	for (int p=0;p<planeEquations.size();p++)
	{
		btVector3 plane = planeEquations[p];
		plane[3] += collisionMargin;
		shiftedPlaneEquations.push_back(plane);
	}
	btAlignedObjectArray<btVector3> shiftedVertices;
	btGeometryUtil::getVerticesFromPlaneEquations(shiftedPlaneEquations,shiftedVertices);
	
	
	convexShape = new btConvexHullShape(&(shiftedVertices[0].getX()),shiftedVertices.size());
	
#else //SHRINK_OBJECT_INWARDS
	
	convexShape = new btConvexHullShape(&(vertices[0].getX()),vertices.size());
#endif 
	
	convexShape->setMargin(0.01f);
	
	delete trimesh;
	
	return make_pair(centroid,convexShape);
}


vector<pair<btVector3, btConvexHullShape*> > ofxBulletConvexDecomposer::decompose(ofMesh &meshToDecompose, btVector3 scale )
{
	assert( meshToDecompose.getMode() == OF_TRIANGLES_MODE );
	int tcount = meshToDecompose.getNumIndices()/3;
	
	// adapted from bullet-2.81-rev2613/Demos/ConvexDecompositionDemo/ConvexDecompositionDemo.cpp
	
	/*
	 unsigned int depth = 5;
	 float cpercent     = 5;
	 float ppercent     = 15;
	 unsigned int maxv  = 16;
	 float skinWidth    = 0.0;
	 
	 // ConvexDecomposition::WavefrontObj wo;
	 ConvexDecomposition::DecompDesc desc;
	 desc.mVcount       = meshToDecompose.getNumVertices();
	 desc.mVertices     = (float*)(meshToDecompose.getVerticesPointer());
	 desc.mTcount       = meshToDecompose.getNumIndices()/3;
	 desc.mIndices      = meshToDecompose.getIndexPointer();
	 desc.mDepth        = depth;
	 desc.mCpercent     = cpercent;
	 desc.mPpercent     = ppercent;
	 desc.mMaxVertices  = maxv;
	 desc.mSkinWidth    = skinWidth;
	 
	 desc.mCallback = this;
	 */
	
	//-----------------------------------------------
	// HACD
	//-----------------------------------------------
	
	std::vector< HACD::Vec3<HACD::Real> > points;
	std::vector< HACD::Vec3<long> > triangles;
	
	for(int i=0; i<meshToDecompose.getNumVertices(); i++ )
	{
		ofVec3f meshVert = meshToDecompose.getVertex(i);
		HACD::Vec3<HACD::Real> vertex( meshVert.x, meshVert.y, meshVert.z );
		points.push_back(vertex);
	}
	
	for(int i=0;i<meshToDecompose.getNumIndices()/3; i+=3 )
	{
		HACD::Vec3<long> triangle(meshToDecompose.getIndex(i+2), meshToDecompose.getIndex(i+1), meshToDecompose.getIndex(i) );
		triangles.push_back(triangle);
	}
	
	
	HACD::HACD myHACD;
	myHACD.SetPoints(&points[0]);
	myHACD.SetNPoints(points.size());
	myHACD.SetTriangles(&triangles[0]);
	myHACD.SetNTriangles(triangles.size());
	myHACD.SetCompacityWeight(0.1);
	myHACD.SetVolumeWeight(0.0);
	
	// HACD parameters
	// Recommended parameters: 2 100 0 0 0 0
	size_t nClusters = 8;
	double concavity = 100;
	bool invert = false;
	bool addExtraDistPoints = false;
	bool addNeighboursDistPoints = false;
	bool addFacesPoints = false;
	
	myHACD.SetNClusters(nClusters);                     // minimum number of clusters
	myHACD.SetNVerticesPerCH(100);                      // max of 100 vertices per convex-hull
	myHACD.SetConcavity(concavity);                     // maximum concavity
	myHACD.SetAddExtraDistPoints(addExtraDistPoints);
	myHACD.SetAddNeighboursDistPoints(addNeighboursDistPoints);
	myHACD.SetAddFacesPoints(addFacesPoints);
	
	myHACD.Compute();
	nClusters = myHACD.GetNClusters();
	
	
	
	vector<pair<btVector3, btConvexHullShape*> > convexShapes;
	for (int c=0;c<nClusters;c++)
	{
		//generate convex result
		size_t nPoints = myHACD.GetNPointsCH(c);
		size_t nTriangles = myHACD.GetNTrianglesCH(c);
		ofLogNotice("ofxBulletConvexDecomposer") << "cluster " << c << " points " << nPoints << " triangles " << nTriangles;
		
		float* vertices = new float[nPoints*3];
		unsigned int* triangles = new unsigned int[nTriangles*3];
		
		HACD::Vec3<HACD::Real> * pointsCH = new HACD::Vec3<HACD::Real>[nPoints];
		HACD::Vec3<long> * trianglesCH = new HACD::Vec3<long>[nTriangles];
		myHACD.GetCH(c, pointsCH, trianglesCH);
		
		// points
		for(size_t v = 0; v < nPoints; v++)
		{
			vertices[3*v] = pointsCH[v].X();
			vertices[3*v+1] = pointsCH[v].Y();
			vertices[3*v+2] = pointsCH[v].Z();
		}
		// triangles
		for(size_t f = 0; f < nTriangles; f++)
		{
			triangles[3*f] = trianglesCH[f].X();
			triangles[3*f+1] = trianglesCH[f].Y();
			triangles[3*f+2] = trianglesCH[f].Z();
		}
		
	
		ConvexResult r(nPoints, vertices, nTriangles, triangles);
		convexShapes.push_back( createConvexHullShapeFromConvexResult(r, scale) );
		
		delete [] pointsCH;
		delete [] trianglesCH;
		delete [] vertices;
		delete [] triangles;
	}

	return convexShapes;
}

vector<ofxBulletBaseRigidShape*> ofxBulletConvexDecomposer::createAndAddShapesForComponents( btDynamicsWorld* world, vector<pair<btVector3, btConvexHullShape*> > components ) {
	vector<ofxBulletBaseRigidShape*> shapes;
	for ( int i=0; i<components.size(); i++ ) {
		float mass = 1;
		btTransform transform;
		transform.setIdentity();
		transform.setOrigin( components[i].first );
		ofxBulletBaseRigidShape* shape = new ofxBulletBaseRigidShape();
		shape->create(world, components[i].second, transform, mass );
		shape->add();
		
		shapes.push_back(shape);
	}
	return shapes;
}

btCompoundShape* ofxBulletConvexDecomposer::constructCompoundShape( vector<pair<btVector3, btConvexHullShape*> > components )
{
// build up the compound shape from the convex shapes
	btCompoundShape* compound = new btCompoundShape();
	
	// add the components as children
	for (int i=0;i<components.size();i++)
	{
		btTransform trans;
		trans.setIdentity();
		
		btVector3 centroid = components[i].first;
		trans.setOrigin(centroid); 
		btConvexHullShape* convexShape = components[i].second;
		compound->addChildShape(trans,convexShape);
	}
	
	return compound;
}

