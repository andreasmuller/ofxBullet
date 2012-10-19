#include "testApp.h"

#include "MeshHelper.h"

//--------------------------------------------------------------
void testApp::setup() {
	ofSetFrameRate(60);
	ofSetVerticalSync(true);
	ofBackground( 10, 10, 10);
	
	camera.setPosition(ofVec3f(0, -3.f, -40.f));
	camera.lookAt(ofVec3f(0, 0, 0), ofVec3f(0, -1, 0));
	
	//camera.cacheMatrices(true);
	
	world.setup();
	world.enableGrabbing();
	world.setCamera(&camera);
	world.setGravity( ofVec3f(0, 25., 0) );
	
	ofVec3f startLoc;
	ofPoint dimens;
	boundsWidth = 30.;
	float hwidth = boundsWidth*.5;
	float depth = 2.;
	float hdepth = depth*.5;
	boundsShape = new ofxBulletCustomShape();
	boundsShape->create(world.world, ofVec3f(0, 0, 0), 10.);
	
	for(int i = 0; i < 6; i++) {
		bounds.push_back( new ofxBulletBox() );
		if(i == 0) { // ground //
			startLoc.set( 0., hwidth+hdepth, 0. );
			dimens.set(boundsWidth, depth, boundsWidth);
		} else if (i == 1) { // back wall //
			startLoc.set(0, 0, hwidth+hdepth);
			dimens.set(boundsWidth, boundsWidth, depth);
		} else if (i == 2) { // right wall //
			startLoc.set(hwidth+hdepth, 0, 0.);
			dimens.set(depth, boundsWidth, boundsWidth);
		} else if (i == 3) { // left wall //
			startLoc.set(-hwidth-hdepth, 0, 0.);
			dimens.set(depth, boundsWidth, boundsWidth);
		} else if (i == 4) { // ceiling //
			startLoc.set(0, -hwidth-hdepth, 0.);
			dimens.set(boundsWidth, depth, boundsWidth);
		} else if (i == 5) { // front wall //
			startLoc.set(0, 0, -hwidth-hdepth);
			dimens.set(boundsWidth, boundsWidth, depth);
		}
		btBoxShape* boxShape = ofBtGetBoxCollisionShape( dimens.x, dimens.y, dimens.z );
		boundsShape->addShape( boxShape, startLoc );
		
		bounds[i]->create( world.world, startLoc, 0., dimens.x, dimens.y, dimens.z );
		bounds[i]->setProperties(.25, .95);
		bounds[i]->add();
	}
	
	bDropBox	= false;
	bDrawDebug	= false;
	
	// this model is huge! scale it down 
	ofVec3f scale(.0009, .0009, .0009);
	
	// 3D logo by Lauren Licherdell | http://www.laurenlicherdell.com/ // 
	assimpModel.loadModel("OFlogo.dae", false);
	assimpModel.setScale(scale.x, scale.y, scale.z);
	assimpModel.setPosition(0, 0, 0);
	logoMesh = assimpModel.getMesh(0);

	float ringScale = 5;
	ringModel.loadModel("blob.dae", false );
	ringModel.setPosition(0,0,0);
	ringMesh = ringModel.getMesh(0);
	ofMesh ringMesh_base = ringModel.getMesh(1);
	MeshHelper::fuseNeighbours( ringMesh );
	MeshHelper::fuseNeighbours( ringMesh_base );
	MeshHelper::appendMesh( ringMesh, ringMesh_base, true );
	
	float ringMass = 3.f;
	
	ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
	
	// let's make an object for the light to follow //
	shapes.push_back( new ofxBulletSphere() );
	((ofxBulletSphere*)shapes[0])->create(world.world, ofVec3f(0, -hwidth+5, -5), .15f, 2.);
	((ofxBulletSphere*)shapes[0])->setSphereResolution( 10 );
	((ofxBulletSphere*)shapes[0])->setActivationState( DISABLE_DEACTIVATION );
	shapes[0]->add();
	
	
	for (int i = 0; i < 0; i++) {
		logos.push_back( new ofxBulletCustomShape() );
		startLoc = ofVec3f( ofRandom(-5, 5), ofRandom(0, -hwidth+5), ofRandom(-5, 5) );
		
		if(i == 0) {
			for(int i = 0; i < assimpModel.getNumMeshes(); i++) {
				logos[i]->addMesh(assimpModel.getMesh(i), scale, true);
			}
		} else {
			logos[i]->init( (btCompoundShape*)logos[0]->getCollisionShape(), logos[0]->getCentroid() );
		}
		logos[i]->create( world.world, startLoc, startRot, 3. );
		logos[i]->add();
	}
	
	for ( int i=0; i<0; i++ )
	{
		shapes.push_back( new ofxBulletCapsule() );
		startLoc = ofVec3f( ofRandom(-5, 5), ofRandom(0, -hwidth+5), ofRandom(-5, 5) );

		float mass = 1.0f;
		float radius = 1.f;
		float height = 2.f;
		((ofxBulletCapsule*)shapes.back())->create( world.world, startLoc, mass, radius, height );
		((ofxBulletCapsule*)shapes.back())->setActivationState( DISABLE_DEACTIVATION );
		shapes.back()->add();
	}
	
	for ( int i=0; i<1; i++ )
	{
		softShapes.push_back( new ofxBulletBaseSoftShape() );

		startLoc = ofVec3f( ofRandom(-5, 5), ofRandom(0, -hwidth+5), ofRandom(-5, 5) );
		
		/*
		ofBuffer faceFile = ofBufferFromFile( "blob.1.face" );
		ofBuffer nodeFile = ofBufferFromFile( "blob.1.node" );
		ofBuffer eleFile = ofBufferFromFile( "blob.1.ele" );
		float tetraMass = 0.3f;
		float tetraScale = 5;
		softShapes.back()->create( world.world, eleFile, faceFile, nodeFile, ofGetBtTransformFromVec3f(startLoc), tetraMass, tetraScale );*/

		softShapes.back()->create( world.world, ringMesh, ofGetBtTransformFromVec3f(startLoc), ringMass, ringScale );
		softShapes.back()->add();
	}
	
	ofSetSmoothLighting(true);
	float lightScale = 0.1f;
	light.setAmbientColor(ofColor(.0, .0, .0));
	light.setDiffuseColor(ofColor(128*lightScale, 128*lightScale, 128*lightScale));
	light.setSpecularColor(ofColor(192*lightScale, 180*lightScale, 180*lightScale ));
	light.setAttenuation(0, 0.5, 0);
	
	float keyScale = 0.5f;
	keyLight.setPosition( -15, -10, -40 );
	keyLight.setAmbientColor(ofColor(.0, .0, .0));
	keyLight.setDiffuseColor(ofColor(192*keyScale, 192*keyScale, 128*keyScale));
	keyLight.setSpecularColor(ofColor(192*keyScale, 180*keyScale, 180*keyScale ));
	//	keyLight.setAttenuation(0,1,0);
	
	float fillScale = 0.3f;
	fillLight.setPosition( 15, 20, -40 );
	fillLight.setAmbientColor(ofColor(.0, .0, .0));
	fillLight.setDiffuseColor(ofColor(192*fillScale, 128*fillScale, 128*fillScale));
	fillLight.setSpecularColor(ofColor(192*fillScale, 180*fillScale, 180*fillScale ));
	//	keyLight.setAttenuation(0,1,0);
	
	
	
	logoMat.setAmbientColor(ofFloatColor(0, 0, 0));
	logoMat.setDiffuseColor(ofFloatColor(150, 0, 150));
	logoMat.setSpecularColor(ofFloatColor(220, 0, 220));
	logoMat.setShininess(40);
	
	boundsMat.setAmbientColor(ofFloatColor(10, 9, 10));
	boundsMat.setDiffuseColor(ofFloatColor(12, 10, 12));
	boundsMat.setSpecularColor(ofFloatColor(1, 1, 1));
	boundsMat.setShininess(10);
	
	shapesMat.setShininess(80);
}

//--------------------------------------------------------------
void testApp::update() {
	
	if(bDropBox && bounds.size() > 0) {
		for (int i = 0; i < bounds.size(); i++) {
			delete bounds[i];
		}
		bounds.clear();
		boundsShape->add();
	}
	
	if(bDropBox) {
		ofVec3f diff = ofVec3f(0, -5, 0) - boundsShape->getPosition();
		diff *= 200.f;
		boundsShape->applyCentralForce(diff);
	}
	
	world.update();
	ofSetWindowTitle(ofToString(ofGetFrameRate(), 0));
	
}

//--------------------------------------------------------------
void testApp::draw() {
	glEnable( GL_DEPTH_TEST );
	
	camera.begin();
	
	ofSetLineWidth(1.f);
	if(bDrawDebug) world.drawDebug();
	
	ofEnableLighting();
	light.enable();
	keyLight.enable();
	//fillLight.enable();
	light.setPosition(shapes[0]->getPosition());
	ofSetColor(255, 255, 255);
	shapes[0]->draw();
	
	ofSetColor(100., 100., 100.);
	
	if(!bDropBox) {
		boundsMat.begin();
		for(int i = 0; i < bounds.size()-1; i++) {
			bounds[i]->draw();
		}
		boundsMat.end();
	} else {
		ofNoFill();
		btScalar	m[16];
		ofGetOpenGLMatrixFromRigidBody( boundsShape->getRigidBody(), m );
		glPushMatrix(); 
		glMultMatrixf( m );
		ofBox(ofVec3f(0, 0,0), boundsWidth);
		glPopMatrix();
		ofFill();
	}
	
	ofDisableAlphaBlending();
	ofDisableBlendMode();
	
	glPushAttrib(GL_ALL_ATTRIB_BITS);
    glPushClientAttrib(GL_CLIENT_ALL_ATTRIB_BITS);
    glEnable(GL_NORMALIZE);
    glDisable(GL_CULL_FACE);
	ofPoint scale		= assimpModel.getScale();
	
	ofSetColor(0, 0.5, 0);
	logoMat.begin();
	for(int i = 0; i < logos.size(); i++) {
		btScalar	m[16];
		ofGetOpenGLMatrixFromRigidBody( logos[i]->getRigidBody(), m );
		glPushMatrix(); 
		glMultMatrixf( m );
		glTranslatef(-logos[i]->getCentroid().x, -logos[i]->getCentroid().y, -logos[i]->getCentroid().z);
		ofScale(scale.x,scale.y,scale.z);
		logoMesh.drawFaces();
		glPopMatrix();
	}
	logoMat.end();

	ofMaterial softMat;
	softMat.setColors( ofColor::gray, ofColor(16, 16, 16), ofColor::white, ofColor::black);
	softMat.setShininess( 0.2f );
	softMat.begin();
	
	for(int i = 0; i < softShapes.size(); i++) {
		softShapes[i]->draw();
	}
	softMat.end();
	glPopAttrib();
	
	ofSetColor(15,197,138);
	ofPushStyle();
	shapesMat.begin();
	for(int i = 0; i < shapes.size(); i++) {
		shapes[i]->draw();
	}
	shapesMat.end();
	ofPopStyle();
	
	light.disable();
	ofDisableLighting();
	
	camera.end();
	glDisable(GL_DEPTH_TEST);
	
	int totalShapes = shapes.size() + logos.size();
	ofVec3f gravity = world.getGravity();
	stringstream ss;
	ss << "seed :" << randSeed << endl;
	ss << "Draw Debug (d): " << bDrawDebug << endl;
	ss << "Total Shapes: " << totalShapes << endl;
	ss << "Add logos(o)" << endl;
	ss << "add spherers (s)" << endl;
	ss << "add boxes (b)" << endl;
	ss << "Gravity(up/down/left/right): x=" << gravity.x << " y= " << gravity.y << " z= " << gravity.z << endl;
	ofSetColor(255, 255, 255);
	ofDrawBitmapString(ss.str().c_str(), 20, 20);
}

//--------------------------------------------------------------
void testApp::keyPressed(int key) {
	ofVec3f mouseLoc = camera.screenToWorld( ofVec3f((float)ofGetMouseX(), (float)ofGetMouseY(), 0) );
	ofVec3f gravity = world.getGravity();
	ofQuaternion startRot = ofQuaternion(1., 0., 0., PI);
	float rsize = ofRandom(.3, 1.8);
	mouseLoc.z += 15;
	switch (key) {
		case 'd':
			bDrawDebug = !bDrawDebug;
			break;
		case 'o':
			logos.push_back( new ofxBulletCustomShape() );
			logos[logos.size()-1]->init( (btCompoundShape*)logos[0]->getCollisionShape(), logos[0]->getCentroid() );
			logos[logos.size()-1]->create( world.world, mouseLoc, startRot, 3. );
			logos[logos.size()-1]->add();
			break;
		case 's':
			shapes.push_back( new ofxBulletSphere() );
			((ofxBulletSphere*)shapes[shapes.size()-1])->create( world.world, mouseLoc, rsize*.2, rsize );
			((ofxBulletSphere*)shapes[shapes.size()-1])->setSphereResolution( 16 );
			((ofxBulletSphere*)shapes[shapes.size()-1])->setActivationState( DISABLE_DEACTIVATION );
			shapes[shapes.size()-1]->add();
			break;
		case 'b':
			shapes.push_back( new ofxBulletBox() );
			((ofxBulletBox*)shapes[shapes.size()-1])->create( world.world, mouseLoc, rsize*.2, rsize*2, rsize*2, rsize*2 );
			((ofxBulletBox*)shapes[shapes.size()-1])->setActivationState( DISABLE_DEACTIVATION );
			shapes[shapes.size()-1]->add();
			break;
		case ' ':
			bDropBox = true;
			break;
		case OF_KEY_UP:
			gravity.y -= 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_DOWN:
			gravity.y += 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_RIGHT:
			gravity.x += 5.;
			world.setGravity( gravity );
			break;
		case OF_KEY_LEFT:
			gravity.x -= 5.;
			world.setGravity( gravity );
			break;
		default:
			break;
	}
}

//--------------------------------------------------------------
void testApp::keyReleased(int key) {
	
}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y) {
	
}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button) {
	
}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){
	
}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h) {
	
}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg) {
	
}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo) { 
	
}