#pragma once

#include "ofMain.h"
#include "btBulletDynamicsCommon.h"

class ofxBulletBaseRigidShape;
class ofxBulletCustomShape;

class ofxBulletConvexDecomposer
{
public:
	
	/// returns vector of pairs< centroid, shape >; caller owns shapes
	static vector<pair<btVector3, btConvexHullShape*> >  decompose(const ofMesh &meshToDecompose, btVector3 scale);
	/// returns a new btCompoundShape by adding everything from components as a child; caller owns the result.
	static btCompoundShape* constructCompoundShape( vector<pair<btVector3, btConvexHullShape*> > components );
	
	static ofxBulletCustomShape* createAndAddCustomShape( btDynamicsWorld* world, btTransform transform, vector<pair<btVector3, btConvexHullShape*> > components );
	
	static vector<ofxBulletBaseRigidShape*> createAndAddShapesForComponents( btDynamicsWorld* world, btTransform transform, vector<pair<btVector3, btConvexHullShape*> > components );
	
private:
};

