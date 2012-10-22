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


class ofxBulletBaseSoftShape  {
public:
	ofxBulletBaseSoftShape();
	~ofxBulletBaseSoftShape();
	
	enum CollisionFilterGroups {
		OFX_BULLET_CUSTOM_SHAPE = 6
	};
	
	/// tetrahedron version
	virtual void createFromTetraBuffer( btSoftRigidDynamicsWorld* a_world, ofBuffer& eleFile, ofBuffer& faceFile, ofBuffer& nodeFile, btTransform a_bt_tr, float mass, float scale, float springStrength=1.0f);
	virtual void createFromOfMesh( btSoftRigidDynamicsWorld* a_world, const ofMesh& a_mesh, btTransform a_bt_tr, float a_mass, float scale );
	
	virtual void add();
		
	void	remove();
	void	removeSoftBody();
	
	
	// move the node with the given index to the given position. timeStep is the last simulation time step
	void moveNode( int nodeIndex, ofVec3f pos, float timeStep );
	
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
	
	float getMass() const;
	void setMass( float mass );
	
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
	
	const btAlignedObjectArray<btSoftBody::Node>& getNodes() { return _softBody->m_nodes; }
	vector<ofVec3f> getNodeLocations();
	void addNode( ofVec3f pos, float mass );
	void addLink( int index0, int index1, btSoftBody::Material* material = 0/*, float linkStrength = 1.0f*/ );
	
	/*
	 const btCylinderShape* cylinder = static_cast<const btCylinderShape*>(shapes[Body::UPPER]);
	 */
protected:
	
	btSoftRigidDynamicsWorld*	_world;
	btSoftBody*					_softBody;
	float						_mass;
	
	void*						_userPointer;
	
	bool						_bInited;
	bool						_bCreated;
	bool						_bAdded;
	
};



