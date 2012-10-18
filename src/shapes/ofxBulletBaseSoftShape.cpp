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

void ofxBulletBaseSoftShape::create( btSoftRigidDynamicsWorld* a_world, const ofMesh& mesh, btTransform a_bt_tr, float a_mass, float scale ){
	
	if ( mesh.getMode() != OF_PRIMITIVE_TRIANGLES ) {
		ofLogError("ofxBulletBaseSoftShape") << "create needs mesh with mode OF_PRIMITIVE_TRIANGLES";
		return;
	}

	if(a_world == NULL) {
		ofLog(OF_LOG_ERROR, "ofxBulletSphere :: create : a_world param is NULL");
		return;
	}
	_mass			= a_mass;
	_world			= a_world;
	
	_bCreated		= true;
	
	
	// convert ofMesh to vertices/triangles list
	vector<ofVec3f> vertices;
	// fuse close-enough vertices
	// first define 'close enough' as 1/10000 of smallest dimension of the mesh
	ofVec3f tlb, brf; // top left back, bottom right front
	for ( int i=0; i<mesh.getNumVertices(); i++ ) {

		ofVec3f vertex = mesh.getVertex(i);
		if ( i == 0 ){
			tlb = brf = vertex;
			continue;
		}

		tlb.x = min(tlb.x,vertex.x);
		tlb.y = min(tlb.y,vertex.y);
		tlb.z = min(tlb.z,vertex.z);
		brf.x = min(brf.x,vertex.x);
		brf.y = min(brf.y,vertex.y);
		brf.z = min(brf.z,vertex.z);
	}
	float minDimension = min(brf.x-tlb.x,min(brf.y-tlb.y, brf.z-tlb.z));
	float fuseDistanceSq = minDimension * 0.0001f * scale;
	fuseDistanceSq *= fuseDistanceSq;
	// now fuse
	map<int,int> fused;
	for ( int i=0; i<mesh.getNumVertices(); i++ )
	{
		ofVec3f vertex = mesh.getVertex(i);
		vertex *= scale;
		//vertex.rotate(10, 10, 10);
		bool didFuse = false;
		for ( int j=0; j<vertices.size(); j++ ){
			if ( (vertex-vertices[j]).lengthSquared()<fuseDistanceSq ){
				// fuse i to j
				fused[i] = j;
				didFuse = true;
				break;
			}
		}
		if ( !didFuse ) {
			vertices.push_back( vertex );
			fused[i] = i;
		}
	}
	
	// we have fused all vertices together, and now the fused map maps from old indices to fuse targets
	vector<int> triangles;
	triangles.resize( mesh.getNumVertices() );
	for ( int i=0; i<mesh.getNumVertices(); i++ )
	{
		// normally triangles would just be ((1,2,3),(4,5,6),(7,8,9),...)
		// but we have fused vertices, so instead of 1,2,3 lookup fused[1],fused[2],fused[3] etc
		triangles.push_back( fused[i] );
	}
	
	int numTriangles = triangles.size()/3;
	/*_softBody		= btSoftBodyHelpers::CreateFromTriMesh( a_world->getWorldInfo(), &(vertices[0].x), &triangles[0], numTriangles );*/
	
	_softBody = btSoftBodyHelpers::CreateFromTriMesh( a_world->getWorldInfo(), gVerticesBunny,
													 &gIndicesBunny[0][0],
													 BUNNY_NUM_TRIANGLES);
	_softBody->scale( btVector3(5, 5, 5));
	setProperties(.4, .75);
	
	_softBody->generateBendingConstraints(2);
	//_softBody->generateClusters(0);
	_softBody->m_cfg.kKHR = 1.0f; // penetration with kinetic
	_softBody->m_cfg.kCHR = 0.8; // penetration
	_softBody->m_cfg.kPR = 0.5f; // pressure
	_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
	_softBody->m_cfg.collisions|=btSoftBody::fCollision::VF_SS;
//	_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
/*	_softBody->generateClusters(5);
	_softBody->m_cfg.piterations	=	2;
	_softBody->m_cfg.kDF			=	0.5;
	_softBody->m_cfg.collisions|=btSoftBody::fCollision::CL_SS;*/
	_softBody->transform(a_bt_tr);
	_softBody->setTotalMass(a_mass,true);

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




