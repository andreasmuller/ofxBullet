/*
 *  ofxBulletBaseCollisionShape.cpp
 *  ofxBullet_v3
 *
 *  Created by Nick Hardeman on 5/18/11.
 *  Copyright 2011 Arnold Worldwide. All rights reserved.
 *
 */

#include "ofxBulletBaseSoftShape.h"
#include "BulletSoftBody/btSoftBodyHelpers.h"
#include "Bunny.h"

//--------------------------------------------------------------
ofxBulletBaseSoftShape::ofxBulletBaseSoftShape() {
	_world			= NULL;
	_softBody		= NULL;
	_bCreated		= false;
	_bInited		= false;
	_bAdded			= false;
	_userPointer	= NULL;
}

//--------------------------------------------------------------
ofxBulletBaseSoftShape::~ofxBulletBaseSoftShape() {
	remove();
}


void ofxBulletBaseSoftShape::create( btSoftRigidDynamicsWorld* a_world, ofBuffer& eleFile, ofBuffer& faceFile, ofBuffer& nodeFile, btTransform a_bt_tr, float a_mass, float scale ) {
	
	_mass = a_mass;
	_world = a_world;

	bool makeFaceLinks = true;
	bool makeTetraLinks = true;
	bool makeFacesFromTetras = false; // has no effect anyway
	_softBody = btSoftBodyHelpers::CreateFromTetGenData( a_world->getWorldInfo(), eleFile.getBinaryBuffer(), faceFile.getBinaryBuffer(),
														nodeFile.getBinaryBuffer(),  
														makeFaceLinks, makeTetraLinks, makeFacesFromTetras );
//	_softBody = btSoftBodyHelpers::CreateFromTetGenData( a_world->getWorldInfo(), TetraBunny::getElements(), 0, TetraBunny::getNodes(), false, true, true );
	
	// ok, it seems we have to manually add faces
	stringstream ss( faceFile.getBinaryBuffer() );
	int numFaces;
	ss >> numFaces;
	int hasBounds;
	ss >> hasBounds;
	for ( int i=0; i<numFaces; i++ ) {
		int faceIndex, n0, n1, n2;
		ss >> faceIndex;
		ss >> n0;
		ss >> n1;
		ss >> n2;
		ofLogNotice("ofxBullBaseSoftShape") << faceIndex << " " << n0 << " " << n1 << " " << n2;
		_softBody->appendFace(n2, n1, n0);
		if ( makeFaceLinks ) {
			_softBody->appendLink( n0, n1, 0, true );
			_softBody->appendLink( n1, n2, 0, true );
			_softBody->appendLink( n2, n0, 0, true );
		}
	}
	
	_softBody->scale( btVector3(scale, scale, scale) );
	
	_bCreated = true;
	//_softBody->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS | btSoftBody::fCollision::CL_RS;

	_softBody->m_cfg.kKHR = 1.0f; // penetration with kinetic
	//_softBody->m_cfg.piterations = 2;
	_softBody->setVolumeMass( a_mass );
	_softBody->generateBendingConstraints(3);
	_softBody->generateClusters(0);
	
	_softBody->transform( a_bt_tr );
	_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
	_softBody->m_cfg.collisions |= btSoftBody::fCollision::CL_SELF;

	
	setProperties(.4, .75);

}


void ofxBulletBaseSoftShape::create( btSoftRigidDynamicsWorld* a_world, const ofMesh& mesh, btTransform a_bt_tr, float a_mass, float scale ){
	
	if ( mesh.getMode() != OF_PRIMITIVE_TRIANGLES ) {
		ofLogError("ofxBulletBaseSoftShape") << "create needs mesh with mode OF_PRIMITIVE_TRIANGLES";
		return;
	}

	if(a_world == NULL) {
		ofLog(OF_LOG_ERROR, "ofxBulletBaseSoftShape :: create : a_world param is NULL");
		return;
	}
	_mass			= a_mass;
	_world			= a_world;
	
	_bCreated		= true;
	
	 
	vector<ofVec3f> vertices = mesh.getVertices();

	
	vector<int> triangles;
	for ( int i=0; i<mesh.getNumIndices(); i++ )
		triangles.push_back( mesh.getIndex(i) );
	
	int numTriangles = triangles.size()/3;
	_softBody		= btSoftBodyHelpers::CreateFromTriMesh( a_world->getWorldInfo(), &(vertices[0].x), &triangles[0], numTriangles );
	_softBody->scale( btVector3(scale, scale, scale));
	setProperties(.4, .75);
	
	_softBody->generateBendingConstraints(2);
	//_softBody->generateClusters(0);
	_softBody->m_cfg.kKHR = 1.0f; // penetration with kinetic
	_softBody->m_cfg.kCHR = 0.8; // penetration
	_softBody->m_cfg.kPR = 1.0f; // pressure
	_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
	_softBody->m_cfg.collisions|=btSoftBody::fCollision::VF_SS;
//	_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
/*	_softBody->generateClusters(5);
	_softBody->m_cfg.piterations	=	2;
	_softBody->m_cfg.kDF			=	0.5;
	_softBody->m_cfg.collisions|=btSoftBody::fCollision::CL_SS;*/
	_softBody->transform(a_bt_tr);
	_softBody->setTotalMass(a_mass,false);
	
	_softBody->randomizeConstraints();
/*	btMatrix3x3	m;
	m.setEulerZYX(a.x(),a.y(),a.z());
	psb->transform(btTransform(m,x));
	psb->scale(btVector3(2,2,2));
	psb->setTotalMass(50,true);
	pdemo->getSoftDynamicsWorld()->addSoftBody(psb);*/

	//setDamping( .25 );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::add() {
	_bAdded = true;
	_world->addSoftBody( _softBody );
	_world->getWorldInfo().m_sparsesdf.Reset();
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::remove() {
	if(_userPointer != NULL) {
		delete _userPointer;
		_userPointer = NULL;
	}
	removeSoftBody();
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::removeSoftBody() {
	if(_world != NULL && _bAdded) {
		//cout << "ofxBulletBaseSoftShape :: removeRigidBody : calling remove rigid body" << endl;
		_world->removeSoftBody(_softBody);
		delete _softBody;
		_softBody = NULL;
		
	}
	_bCreated = _bInited = _bAdded = false;
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator==( const void* a_userData) const {
	return getData() == a_userData;
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator==( const ofxBulletCollisionData& a_collisionData) const {
	return (getData() == a_collisionData.userData1) || (getData() == a_collisionData.userData2);
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator!=( const ofxBulletCollisionData& a_collisionData) const {
	return (getData() != a_collisionData.userData1) && (getData() != a_collisionData.userData2);
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator==( const ofxBulletMousePickEvent& a_e ) const {
	return getData() == a_e.userData;
}
//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator!=( const ofxBulletMousePickEvent& a_e ) const {
	return !(getData() == a_e.userData);
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator==( const ofxBulletRaycastData& a_e ) const {
	return getData() == a_e.userData;
}
//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::operator!=( const ofxBulletRaycastData& a_e ) const {
	return !(getData() == a_e.userData);
}




// GETTERS //

//--------------------------------------------------------------
btSoftBody* ofxBulletBaseSoftShape::getSoftBody() {
	return _softBody;
}

//--------------------------------------------------------------
void* ofxBulletBaseSoftShape::getData() const {
	return _softBody->getUserPointer();
}


//--------------------------------------------------------------
int ofxBulletBaseSoftShape::getActivationState() {
	// returns OF_BT_ACTIVATION_STATE_ACTIVE || OF_BT_ACTIVATION_ISLAND_SLEEPING
	return ((btCollisionObject*)_softBody->getCollisionShape())->getActivationState();
}

//--------------------------------------------------------------
int ofxBulletBaseSoftShape::getType() {
	return _type;
}


//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getMass() const {
	return _mass;
}

/*//--------------------------------------------------------------
void ofxBulletBaseSoftShape::getOpenGLMatrix( btScalar* a_m ) {
	ofGetOpenGLMatrixFromRigidBody( _softBody, a_m );
}

//--------------------------------------------------------------
ofMatrix4x4 ofxBulletBaseSoftShape::getTransformationMatrix() const {
	float	m[16];
	ofGetOpenGLMatrixFromRigidBody( _softBody, m );
	ofMatrix4x4 mat;
	mat.set(m[0], m[1], m[2], m[3],
			m[4], m[5], m[6], m[7],
			m[8], m[9], m[10], m[11],
			m[12], m[13], m[14], m[15]);
	return mat;
}*/

/*
//--------------------------------------------------------------
ofVec3f ofxBulletBaseSoftShape::getPosition() const {
	return ofGetVec3fPosFromRigidBody( _softBody );
}

// returns yaw, pitch, roll //
//--------------------------------------------------------------
ofVec3f ofxBulletBaseSoftShape::getRotation( ) const {
	return ofGetRotationFromRigidBody( _softBody );
}

//--------------------------------------------------------------
ofVec3f ofxBulletBaseSoftShape::getRotationAxis() const {
	btQuaternion rotQuat		= _softBody->getWorldTransform().getRotation();
	btVector3 btaxis			= rotQuat.getAxis();
	return ofVec3f( btaxis.getX(), btaxis.getY(), btaxis.getZ() );
}

//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getRotationAngle() const {
	btQuaternion rotQuat		= _softBody->getWorldTransform().getRotation();
	return rotQuat.getAngle();
}

//--------------------------------------------------------------
ofQuaternion ofxBulletBaseSoftShape::getRotationQuat() const {
	ofVec3f axis	= getRotationAxis();
	return ofQuaternion( axis.x, axis.y, axis.z, getRotationAngle());
}

*/


//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getRestitution() const {
	return (float)_softBody->getRestitution();
}

//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getFriction() const {
	return _softBody->getFriction();
}

/*
//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getDamping() const {
	return (float)_softBody->getLinearDamping();
}

//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getAngularDamping() const {
	return (float)_softBody->getAngularDamping();
}
 */



/**************************************************************/
// PROPERTY SETTERS, must be called after create() and before add() //

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setProperties(float a_restitution, float a_friction) {
	setRestitution(a_restitution);
	setFriction(a_friction);
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setRestitution( float a_res ) {
	if(checkCreate()) {
		_softBody->setRestitution( btScalar(a_res) );
	}
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setFriction( float a_friction ) {
	if(checkCreate()) {
		_softBody->setFriction( btScalar(a_friction) );
	}
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setActivationState( int a_state ) {
	if(checkCreate()) {
		_softBody->setActivationState( a_state );
	}
}
/**************************************************************/




// SETTERS, may be called after create() //
//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setData(void* userPointer) {
	_userPointer = userPointer;
	_softBody->setUserPointer( _userPointer );
}

/*
//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setDamping( float a_linear_damp ) {
	setDamping( a_linear_damp, getAngularDamping() );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setAngularDamping( float a_angular_damp ) {
	setDamping( getDamping(), a_angular_damp );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::setDamping( float a_linear_damp, float a_angular_damp ) {
	_softBody->setDamping(a_linear_damp, a_angular_damp);
}

*/



// CHECKERS //
//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::checkInit() {
	if(!_bInited) {
		ofLog(OF_LOG_WARNING, "ofxBulletBaseSoftShape :: must call init() first!");
		return false;
	}
	return true;
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::checkPropCreate() {
	if(_bCreated) {
		ofLog(OF_LOG_WARNING, "ofxBulletBaseSoftShape :: these property changes will not affect the shape, must call before create()!");
		return true;
	}
	return false;
}

//--------------------------------------------------------------
bool ofxBulletBaseSoftShape::checkCreate() {
	if(!_bCreated) {
		ofLog(OF_LOG_WARNING, "ofxBulletBaseSoftShape :: must call create() first");
		return false;
	}
	return true;
}




//--------------------------------------------------------------
void ofxBulletBaseSoftShape::activate() {
	//((btCollisionObject*)_softBody->getCollisionShape())->activate( true );
	getSoftBody()->activate( true );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::enableKinematic() {
	getSoftBody()->setCollisionFlags( getSoftBody()->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT );
}

/*//--------------------------------------------------------------
void ofxBulletBaseSoftShape::applyForce( const ofVec3f& a_frc, const ofVec3f& a_rel_pos ) {
	_softBody->applyForce( btVector3(a_frc.x, a_frc.y, a_frc.z), btVector3(a_rel_pos.x, a_rel_pos.y, a_rel_pos.z) );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::applyForce( const btVector3& a_frc, const btVector3& a_rel_pos ) {
	_softBody->applyForce( a_frc, a_rel_pos );
}*/

/*
//--------------------------------------------------------------
void ofxBulletBaseSoftShape::applyCentralForce( const ofVec3f& a_frc ) {
	applyCentralForce( btVector3(a_frc.x, a_frc.y, a_frc.z) );
}
void ofxBulletBaseSoftShape::applyCentralForce( float a_x, float a_y, float a_z ) {
	applyCentralForce( btVector3(a_x, a_y, a_z) );
}
void ofxBulletBaseSoftShape::applyCentralForce( const btVector3& a_frc ) {
	_softBody->applyCentralForce( a_frc );
}

//--------------------------------------------------------------
void ofxBulletBaseSoftShape::applyTorque( const ofVec3f& a_torque ) {
	applyTorque( btVector3(a_torque.x, a_torque.y, a_torque.z) );
}
void ofxBulletBaseSoftShape::applyTorque( float a_x, float a_y, float a_z ) {
	applyTorque( btVector3( a_x, a_y, a_z ) );
}
void ofxBulletBaseSoftShape::applyTorque( const btVector3& a_torque ) {
	_softBody->applyTorque( a_torque );
}

*/



void ofxBulletBaseSoftShape::draw(){

	/* Links	*/
//	ofSetColor( ofColor::white );

	ofMesh mesh;
	for ( int i=0; i<_softBody->m_faces.size();++i )
	{
		const btSoftBody::Face face = _softBody->m_faces[i];
		int vertIndex = mesh.getNumVertices();

		ofVec3f vertices[3] = {
			ofVec3f(face.m_n[0]->m_x.getX(), face.m_n[0]->m_x.getY(), face.m_n[0]->m_x.getZ()),
			ofVec3f(face.m_n[1]->m_x.getX(), face.m_n[1]->m_x.getY(), face.m_n[1]->m_x.getZ()),
			ofVec3f(face.m_n[2]->m_x.getX(), face.m_n[2]->m_x.getY(), face.m_n[2]->m_x.getZ()),
		};
			
		ofVec3f normal = (vertices[2]-vertices[1]).cross(vertices[0]-vertices[1]).normalize();
		for ( int j=0; j<3; j++ ) {
			mesh.addVertex( vertices[j] );
			mesh.addNormal( normal );
		}
		mesh.addTriangle( vertIndex, vertIndex+1, vertIndex+2 );
	}
	mesh.drawFaces();
	
	/*
	mesh.clear();
	for ( int i=0; i<_softBody->m_nodes.size();++i )
	{
		const btSoftBody::Node& n = _softBody->m_nodes[i];
		mesh.addVertex( ofVec3f( n.m_x.getX(), n.m_x.getY(), n.m_x.getZ() ) );
	}
	mesh.drawVertices();
		
	for(int i=0;i<_softBody->m_links.size();++i)
	{
		const btSoftBody::Link&	l=_softBody->m_links[i];
		
		const btVector3& a = l.m_n[0]->m_x;
		const btVector3& b = l.m_n[1]->m_x;
		ofVec3f aOf( a.getX(), a.getY(), a.getZ() );
		ofVec3f bOf( b.getX(), b.getY(), b.getZ() );
		ofLine( aOf, bOf );
	}*/
}




