/*
 *  ofxBulletBaseCollisionShape.h
 *  ofxBullet_v3
 *
 *  Created by Nick Hardeman on 5/18/11.
 *  Copyright 2011 Arnold Worldwide. All rights reserved.
 *
 */

#pragma once
#include "ofMain.h"
#include "btBulletDynamicsCommon.h"
#include "BulletSoftBody/btSoftRigidDynamicsWorld.h"
#include "BulletSoftBody/btSoftBody.h"
#include "ofxBulletCollisionData.h"
#include "ofxBulletConstants.h"
#include "ofxBulletUtils.h"
#include "ofxBulletUserData.h"
#include "ofxBulletMousePickEvent.h"
#include "ofxBulletBaseRigidShape.h"
#include <set>

class ofxBulletBaseSoftShape  {
public:
	ofxBulletBaseSoftShape();
	~ofxBulletBaseSoftShape();
	
	enum CollisionFilterGroups {
		OFX_BULLET_CUSTOM_SHAPE = 6
	};
	
	/// tetrahedron version
	virtual void createFromTetraBuffer( btSoftRigidDynamicsWorld* a_world, ofBuffer& eleFile, ofBuffer& faceFile, ofBuffer& nodeFile, btTransform a_bt_tr,
									   float mass, float scale, float internalSpringStrength=1.0f, float bendingConstraintsSpringStrength=-1, float borderSpringStrength=-1 );
	virtual void createFromOfMesh( btSoftRigidDynamicsWorld* a_world, const ofMesh& a_mesh, btTransform a_bt_tr, float a_mass, float scale );
	virtual void createFromPatch( btSoftRigidDynamicsWorld* a_world, ofVec3f topleft, ofVec3f topright, ofVec3f bottomleft, ofVec3f bottomright, float mass );
	virtual void createFromCuboid( btSoftRigidDynamicsWorld* a_world, ofVec3f topleftback, ofVec3f bottomrightfront, int countX, int countY, int countZ, float totalMass  );

	
	virtual void add();
		
	void	remove();
	void	removeSoftBody();
	
	
	// move the node with the given index to the given position. timeStep is the last simulation time step
	void moveNode( int nodeIndex, ofVec3f pos, float timeStep );
	
	void anchorNode( int nodeIndex, btRigidBody* anchorBody );
	void anchorNode( int nodeIndex, btRigidBody* anchorBody, ofVec3f offset );
	
	// GETTERS //
	btSoftBody*	getSoftBody();
	virtual void*	getData() const;

	
	
	// used for checking collisions. Data is set using getData and operator is used to see if the same //
	bool	operator==( const void* userData) const;
	bool	operator==( const ofxBulletCollisionData& a_collisionData) const;
	bool	operator!=( const ofxBulletCollisionData& a_collisionData) const;
	
	bool	operator==( const ofxBulletMousePickEvent& a_e ) const;
	bool	operator!=( const ofxBulletMousePickEvent& a_e ) const;
	bool	operator==( const ofxBulletRaycastData& a_e ) const;
	bool	operator!=( const ofxBulletRaycastData& a_e ) const;
	
	// PROPERTY SETTERS, must be called after init() and before create() //
	// if you extend this class, must overwrite in your class //
	virtual void setData(void* userPointer);

	void setProperties(float a_restitution, float a_friction);
	
	void setRestitution( float a_res );
	float			getRestitution() const;
	void setFriction( float a_friction );
	float			getFriction() const;

	void setActivationState( int a_state );
	int				getActivationState();

	void setPressure( float pressure ); // -inf..inf
	float getPressure();
	void setDamping( float damping ); // 0..1, 1 = fully damped
	float getDamping();
	
	void setVolumeConservationCoefficient( float coeff );
	float getVolumeConservationCoefficient();
	
	void setTetraPressure( float p );
	float getTetraPressure();
	
	float getMass() const;
	void setMass( float mass );
	
	float getHydrostaticConstant() const;
	void setHydrostaticConstant( float hydroConstant );
	
	float getVolume() const;
	float getRestVolume() const;
	float calculateVolumeOfTetras() const;
	
	void activate();
	// can be called at any time //
	// if you want to control the object. Make sure to have a mass of 0.f
	// and manipulate the rigid body and then call activate() //
	void enableKinematic();
	
	// CHECKERS //
	bool checkInit();
	bool checkPropCreate();
	bool checkCreate();
	

	virtual void draw();
	virtual void drawFaces();
	virtual void drawLinksWithMaterial(int materialIndex);


	// nodes
	void addNode( ofVec3f pos, float mass );
	vector<ofVec3f> getNodeLocations();
	int getNumNodes() { return _softBody->m_nodes.size(); }
	btSoftBody::Node& getNode( int index ) { assert(index>=0 && index<getNumNodes()); return _softBody->m_nodes[index]; }
	int getIndexOfNode( btSoftBody::Node* n );
	bool isNodeExternal( int nodeIndex ) { return externalNodes.count(nodeIndex); }
	
	
	// links
	/// if suppressGenerateClusters is true you need to call generateClusters after adding your links
	/// if the link already exists and replaceMaterial is true, the existing link's material is replaced with the given materialIndex,
	/// if the link already exists and replaceMaterial is false, the function just returns without modifying anything.
	void addLink( int index0, int index1, int materialIndex = 0/*, float linkStrength = 1.0f*/, bool suppressGenerateClusters=false, bool replaceMaterial=false );
	int getNumLinks() { return _softBody->m_links.size(); }
	btSoftBody::Link& getLink( int index ) { assert(index>=0 && index<getNumLinks()); return _softBody->m_links[index]; };
	int getIndexOfLinkBetween( int node0, int node1 );
	btSoftBody::Link& getLinkBetween( int node0, int node1 ) { return getLink( getIndexOfLinkBetween(node0, node1) ); }
	bool hasLinkBetween( int node0, int node1 ) { return getIndexOfLinkBetween( node0, node1 ) != -1; }
	set<int> getAllNeighboursOf( int node );
	set<int> getAllLinksTouching( int nodeIndex );
	
	vector<int> findShortestPath( int fromNode, int toNode );

	// materials
	int getMaterialIndexForLink( int linkIndex );
	void setMaterialIndexForLink( int linkIndex, int matIndex ) { getLink(linkIndex).m_material = _softBody->m_materials[matIndex]; }
	void setMaterialIndexForLink( int nodeIndex0, int nodeIndex1, int matIndex ) { getLinkBetween( nodeIndex0, nodeIndex1 ).m_material = _softBody->m_materials[matIndex]; }
	int addMaterial() { _softBody->appendMaterial(); return _softBody->m_materials.size()-1; }
	void setMaterialSpringConstant( int matIndex, float springConstant ) { _softBody->m_materials[matIndex]->m_kLST = springConstant; _softBody->updateLinkConstants(); }
	float getMaterialSpringConstant( int matIndex ) { return _softBody->m_materials[matIndex]->m_kLST; }
	int getExternalMaterialIndex() { return (_softBody->m_materials.size()>0?1:0); }
	int getBendingConstraintsMaterialIndex() { return (_softBody->m_materials.size()>1?2:0); }
	
	// equalize spring lengths based on material properties
	void equalizeSpringLengths();
	
	
	// perform after adding a link, done automatically unless addLink was called with suppressGenerateClusters==true
	void generateClusters( int clusterSize=0 ) { _softBody->generateClusters( clusterSize ); }
	void generateBendingConstrings( int distance=2 ) { _softBody->generateBendingConstraints( distance, (_softBody->m_materials.size()>2)?_softBody->m_materials[2]:_softBody->m_materials[0]); }

	
	// collisions
	// defaults to false
	void setSelfCollisionsEnabled( bool enable ) { if ( enable ) _softBody->m_cfg.collisions |= btSoftBody::fCollision::CL_SELF; 
												else _softBody->m_cfg.collisions &= ~btSoftBody::fCollision::CL_SELF; }
	// defaults to 0.1
	void setCollisionMargin( float margin ) { _softBody->getCollisionShape()->setMargin( margin ); }
	float getCollisionMargin() { return _softBody->getCollisionShape()->getMargin(); }
	
	/*
	 const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shapes[Body::UPPER]);
	 */
protected:
	
	btSoftRigidDynamicsWorld*	_world;
	btSoftBody*					_softBody;
	
	void*						_userPointer;
	
	bool						_bInited;
	bool						_bCreated;
	bool						_bAdded;
	
	set<int>					externalNodes;
	
	vector<float>				springLengthForceMultipliers;
	
	btSoftBody* createSoftBodyWithTetGenNodes( btSoftBodyWorldInfo& worldInfo, const char* node );
	void appendTetGenFaces( const char* face, bool makeFaceLinks, btSoftBody::Material* linkMaterial );
	void appendTetGenTetras( const char* ele, bool makeTetraLinks, btSoftBody::Material* linkMaterial );
	
	void setClusterCollisionParams();
};



