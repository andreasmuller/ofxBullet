//
//  MeshHelper.cpp
//  ofxBulletSoft
//
//  Created by Damian Stewart on 19.10.12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include "MeshHelper.h"
#include <set>

void MeshHelper::calculateAABoundingBox( const ofMesh& mesh, ofVec3f& topLeftBack, ofVec3f& bottomRightFront )
{
	ofVec3f& tlb = topLeftBack;
	ofVec3f& brf = bottomRightFront;

	for ( int i=0; i<mesh.getNumVertices(); i++ ) {
		
		ofVec3f vertex = mesh.getVertex(i);
		if ( i == 0 ){
			tlb = brf = vertex;
			continue;
		}
		
		tlb.x = min(tlb.x,vertex.x);
		tlb.y = min(tlb.y,vertex.y);
		tlb.z = min(tlb.z,vertex.z);
		brf.x = max(brf.x,vertex.x);
		brf.y = max(brf.y,vertex.y);
		brf.z = max(brf.z,vertex.z);
	}
}

void MeshHelper::fuseNeighbours( ofMesh& mesh, float fuseDistance )
{
	//@todo tex coords, normals
	assert( mesh.getMode() == OF_PRIMITIVE_TRIANGLES );
	int oldNumVerts = mesh.getNumVertices();
	int oldNumIndices = mesh.getNumIndices();
	
	if ( fuseDistance < 0 )
	{
		// fuse close-enough vertices
		// first define 'close enough' as 1/10000 of smallest dimension of the bounding box width/height/depth
		ofVec3f tlb, brf; // top left back, bottom right front
		calculateAABoundingBox( mesh, tlb, brf );
		float minDimension = min(brf.x-tlb.x,min(brf.y-tlb.y, brf.z-tlb.z));
		fuseDistance = minDimension * 0.00001f;
	}
	
	// now fuse
	map<ofIndexType,ofIndexType> fused;
	vector<ofVec3f> newVertices;
	vector<ofIndexType> remove;
	for ( ofIndexType i=0; i<mesh.getNumVertices(); i++ )
	{
		const ofVec3f& vertex = mesh.getVertex(i);
		// look at all the earlier vertices
		bool didFuse = false;
		for ( ofIndexType j=0; j<newVertices.size(); j++ ) {
			if ( (vertex-newVertices[j]).length()<fuseDistance ) {
				// fuse i to j
				fused[i] = j;
				remove.push_back(i);
				didFuse = true;
				break;
			}
		}
		if ( !didFuse ) {
			newVertices.push_back( vertex );
			fused[i] = newVertices.size()-1;
		}
	}
	
	// update indices
	for ( int i=0; i<mesh.getNumIndices(); i++ ) {
		ofIndexType originalIndex = mesh.getIndex(i);
		assert( fused.find( originalIndex ) != fused.end() );
		if ( fused.find(originalIndex) != fused.end() ) {
			mesh.getIndices()[i] = fused[originalIndex];
		}
	}
	
	// remove the fused
	for ( int i=remove.size()-1; i>=0; i-- ) {
		mesh.removeVertex( remove[i] );
	}
		
	ofLogNotice("MeshHelper") << "fuseNeighbours inplace: input " << oldNumVerts << " vertices/" << oldNumIndices << " indices, output " << mesh.getNumVertices() << " vertices/" << mesh.getNumIndices() << " indices";

}

void MeshHelper::fuseNeighbours( ofMesh& outputMesh, const ofMesh& sourceMesh, float fuseDistance )
{
	//@todo tex coords, normals
	assert( sourceMesh.getMode() == OF_PRIMITIVE_TRIANGLES );
	
	if ( fuseDistance < 0 )
	{
		// fuse close-enough vertices
		// first define 'close enough' as 1/10000 of smallest dimension of the bounding box width/height/depth
		ofVec3f tlb, brf; // top left back, bottom right front
		calculateAABoundingBox( sourceMesh, tlb, brf );
		float minDimension = min(brf.x-tlb.x,min(brf.y-tlb.y, brf.z-tlb.z));
		fuseDistance = minDimension * 0.00001f;
	}

	// now fuse
	map<int,int> fused;
	vector<ofVec3f> vertices;
	for ( int i=0; i<sourceMesh.getNumVertices(); i++ )
	{
		const ofVec3f& vertex = sourceMesh.getVertex(i);
		//vertex.rotate(10, 10, 10);
		bool didFuse = false;
		for ( int j=0; j<vertices.size(); j++ ) {
			if ( (vertex-vertices[j]).length()<fuseDistance ) {
				// fuse i to j
				fused[i] = j;
				didFuse = true;
				break;
			}
		}
		if ( !didFuse ) {
			vertices.push_back( vertex );
			fused[i] = vertices.size()-1;
		}
	}
		
	// build the output mesh
	outputMesh.clear();
	outputMesh.addVertices(vertices);
	if ( sourceMesh.getNumIndices() > 0 ) {
		// walk through indices to build up the new mesh
		const vector<ofIndexType>& indices = sourceMesh.getIndices();
		for ( int i=0; i<indices.size(); i+=3 ) {
			assert( fused.find( indices[i] ) != fused.end() );
			assert( fused.find( indices[i+1] ) != fused.end() );
			assert( fused.find( indices[i+2] ) != fused.end() );
			outputMesh.addTriangle( fused[indices[i]], fused[indices[i+1]], fused[indices[i+2]] );
		}
	} else {
		// triangles are just triples of vertices
		for ( int i=0; i<sourceMesh.getNumVertices(); i+=3 ) {
			outputMesh.addTriangle( fused[i], fused[i+1], fused[i+2] );
		}
	}
	
	ofLogNotice("MeshHelper") << "fuseNeighbours: input " << sourceMesh.getNumVertices() << " vertices/" << sourceMesh.getNumIndices() << " indices, output " << outputMesh.getNumVertices() << " vertices/" << outputMesh.getNumIndices() << " indices";
}


void MeshHelper::appendMesh( ofMesh& targetMesh, const ofMesh& toAppend, bool fuse )
{
	ofIndexType indexOffset = targetMesh.getNumVertices();
	targetMesh.addVertices( toAppend.getVertices() );
	// append indices
	const vector<ofIndexType>& indices = toAppend.getIndices();
	for ( int i=0; i<indices.size(); i++ ) {
		targetMesh.addIndex( indices[i]+indexOffset );
	}
	
	if ( fuse )
		fuseNeighbours(targetMesh);
}


void MeshHelper::scaleMesh( ofMesh& mesh, float scale )
{
	for ( int i=0; i<mesh.getNumVertices(); i++ )
		mesh.getVertices()[i] *= scale;
}

void MeshHelper::rotateMesh( ofMesh& mesh, float angle, ofVec3f axis )
{
	for ( int i=0; i<mesh.getNumVertices(); i++ )
		mesh.getVertices()[i].rotate( angle, axis );
}
