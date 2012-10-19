//
//  MeshHelper.h
//  ofxBulletSoft
//
//  Created by Damian Stewart on 19.10.12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#pragma once

#include "ofMain.h"

class MeshHelper
{
public:
	static void calculateAABoundingBox( const ofMesh& theMesh, ofVec3f& topLeftBack, ofVec3f& bottomRightFront );
	static void fuseNeighbours( ofMesh& mesh, float fuseDistance = -1 );
	static void fuseNeighbours( ofMesh& outputMesh, const ofMesh& sourceMesh, float fuseDistance=-1 );
	static void appendMesh( ofMesh& targetMesh, const ofMesh& toAppend, bool fuse=true );
	
	static void scaleMesh( ofMesh& mesh, float scale );
	static void rotateMesh( ofMesh& mesh, float angle, ofVec3f axis );
};

