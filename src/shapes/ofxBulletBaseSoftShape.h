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
	virtual void create( btSoftRigidDynamicsWorld* a_world, ofBuffer& eleFile, ofBuffer& faceFile, ofBuffer& nodeFile, btTransform a_bt_tr, float a_mass, float scale );
	virtual void create( btSoftRigidDynamicsWorld* a_world, const ofMesh& a_mesh, btTransform a_bt_tr, float a_mass, float scale );
	virtual void add();
	void	remove();
	void	removeSoftBody();
	
	// GETTERS //
	btSoftBody*	getSoftBody();
	virtual void*	getData() const;
	btCollisionShape* getCollisionShape() const;
	int				getActivationState();
	
	int				getType();
	
	float			getMass() const;
	void			getOpenGLMatrix( btScalar* a_m );
	ofMatrix4x4		getTransformationMatrix() const;
	ofVec3f			getPosition() const;
	ofVec3f			getRotation() const;
	ofVec3f			getRotationAxis() const;
	float			getRotationAngle() const;
	ofQuaternion	getRotationQuat() const;
	
	float			getRestitution() const;
	float			getFriction() const;
	float			getDamping() const;
	float			getAngularDamping() const;
	
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
	void setFriction( float a_friction );
	void setActivationState( int a_state );
	
	// SETTERS, must be called after create() //
	void setDamping( float a_linear_damp );
	void setAngularDamping( float a_angular_damp );
	void setDamping( float a_linear_damp, float a_angular_damp );
	void activate();
	// can be called at any time //
	// if you want to control the object. Make sure to have a mass of 0.f
	// and manipulate the rigid body and then call activate() //
	void enableKinematic();
	
	// CHECKERS //
	bool checkInit();
	bool checkPropCreate();
	bool checkCreate();
	
	// FORCES //
	void applyForce( const ofVec3f& a_frc, const ofVec3f& a_rel_pos );
	void applyForce( const btVector3& a_frc, const btVector3& a_rel_pos );
	
	void applyCentralForce( const ofVec3f& a_frc );
	void applyCentralForce( float a_x, float a_y, float a_z );
	void applyCentralForce( const btVector3& a_frc );
	
	void applyTorque( const ofVec3f& a_torque );
	void applyTorque( float a_x, float a_y, float a_z );
	void applyTorque( const btVector3& a_torque );
	
	virtual void draw();
	
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
	
	int							_type;
};



