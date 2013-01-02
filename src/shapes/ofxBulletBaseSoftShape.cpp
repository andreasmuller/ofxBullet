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
#include <deque>
#include <queue>

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

vector<int> ofxBulletBaseSoftShape::findShortestPath( int from, int to )
{
	// breadth-first
	typedef struct Ligament
	{
		vector<int> path;
		btVector3 direction;
		int next;
		float fitness;
		Ligament( int start ) { next = start; fitness=1; }
		bool operator<( const Ligament& other) const { return fitness < other.fitness; }
	} Ligament;
	priority_queue<Ligament> queue;
	set<int> visited;
	
	btVector3 startPos = this->getNode(from).m_x;
	queue.push( Ligament(from) );
	
	while ( !queue.empty() )
	{
		// dequeue
		Ligament ligament = queue.top();
		queue.pop();
		int current = ligament.next;
		
		// continue if visited
		if ( visited.find(current) != visited.end() )
		{
			continue;
		}
		
		// visit current
		visited.insert(current);
		// update path
		ligament.path.push_back( current );
		
		// check
		if ( current == to )
		{
			return ligament.path;
		}
		else
		{
			// push all unvisited neighbours to the queue
			set<int> neighbours = this->getAllNeighboursOf( current );
			for ( set<int>::iterator it = neighbours.begin(); it != neighbours.end(); ++it )
			{
				if ( visited.find(*it)==visited.end() )
				{
					ligament.next = *it;
					
					// calculate fitness as path length * direction coefficient
					// where direction coefficient is 1 if newDirection == ligament.direction, -1 if newDirection is opposite ligamentDirection
					ligament.fitness = 1.0f/ligament.path.size();
					btVector3 newDirection = (this->getNode(*it).m_x - startPos).normalized();
					if ( ligament.path.size() < 2 )
					{
						ligament.direction = newDirection;
						ligament.fitness += 1;
					}
					else
					{
						float dirFactor = ligament.direction.dot( newDirection );
						ligament.fitness += dirFactor;
					}
					queue.push( ligament );
				}
			}
		}
	}
	
	// nothing found
	return vector<int>();
}

void ofxBulletBaseSoftShape::createFromCuboid( btSoftRigidDynamicsWorld* a_world, ofVec3f topleftback, ofVec3f bottomrightfront, int countX, int countY, int countZ, float totalMass  )
{

	_world = a_world;

	// from OpenTissue
	/**
	 * t4mesh Block Generator.
	 *
	 * @param I               The number of blocks along x axis.
	 * @param J               The number of blocks along y axis.
	 * @param K               The number of blocks along z axis.
	 * @param block_width     The edgelength of the blocks along x-axis.
	 * @param block_height    The edgelength of the blocks along x-axis.
	 * @param block_depth     The edgelength of the blocks along x-axis.
	 * @param mesh            A generic t4mesh, which upon return holds the generated mesh.
	 */
	
	unsigned int I = countX;
	unsigned int J = countY;
	unsigned int K = countZ;
	float block_width = (bottomrightfront.x-topleftback.x)/countX;
	float block_depth = (bottomrightfront.y-topleftback.y)/countY;
	float block_height = (bottomrightfront.z-topleftback.z)/countZ;
	float oX = topleftback.x;
	float oY = topleftback.y;
	float oZ = topleftback.z;

	{
		float m = 1;
		
		unsigned int numVertices = (I + 1) * (J + 1) * (K + 1);
		btVector3* vertices = new btVector3[numVertices];
		btScalar* masses = new btScalar[numVertices];
		
		unsigned int vertIt = 0;
		for (unsigned int x = 0; x <= I; ++x)
		{
			for (unsigned int y = 0; y <= J; ++y)
			{
				for (unsigned int z = 0; z <= K; ++z)
				{
					vertices[vertIt] = btVector3( oX+block_width*x, oY+block_depth*y, oZ+block_height*z );
					masses[vertIt] = m;
					vertIt++;
				}
			}
		}
		
		_softBody = new btSoftBody(&_world->getWorldInfo(),numVertices,vertices,masses);

		delete[] masses;
		delete[] vertices;
		
		unsigned int fixeds = 1|2|4|8;

#define IDX(_x_,_y_,_z_) (((_x_*(J+1)+_y_)*(K+1))+_z_)
		if(fixeds&1)	_softBody->setMass(IDX(0,J,0),0);
		if(fixeds&2)	_softBody->setMass(IDX(I,J,0),0);
		if(fixeds&4)	_softBody->setMass(IDX(0,J,K),0);
		if(fixeds&8)	_softBody->setMass(IDX(I,J,K),0);
		


		for (unsigned int i = 0; i < I; ++i)
		{
			for (unsigned int j = 0; j < J; ++j)
			{
				for (unsigned int k = 0; k < K; ++k)
				{
					// For each block, the 8 corners are numerated as:
					//     4*-----*7
					//     /|    /|            |  J
					//    / |   / |            |
					//  5*-----*6 |            |
					//   | 0*--|--*3           *------> I
					//   | /   | /            /
					//   |/    |/            / K
					//  1*-----*2
					int p0 = (i * (J + 1) + j) * (K + 1) + k;
					int p1 = p0 + 1;
					int p3 = ((i + 1) * (J + 1) + j) * (K + 1) + k;
					int p2 = p3 + 1;
					int p7 = ((i + 1) * (J + 1) + (j + 1)) * (K + 1) + k;
					int p6 = p7 + 1;
					int p4 = (i * (J + 1) + (j + 1)) * (K + 1) + k;
					int p5 = p4 + 1;
					
					// add faces
					if ( j==J-1 ) {
						_softBody->appendFace( p6, p7, p4 );
						_softBody->appendFace( p4, p5, p6 );
					}
					if ( j==0 )	{
						_softBody->appendFace( p3, p1, p2 );
						_softBody->appendFace( p0, p1, p3 );
					}
					if ( k==K-1 ){
						_softBody->appendFace( p1, p2, p6 );
						_softBody->appendFace( p6, p5, p1 );
					}
					if ( k==0 )	{
						_softBody->appendFace( p4, p7, p3 );
						_softBody->appendFace( p4, p3, p0 );
					}
					
					if ( i==0 )	{
						_softBody->appendFace( p1, p5, p4 );
						_softBody->appendFace( p4, p0, p1 );
					}
					if ( i==I-1 ) {
						_softBody->appendFace( p6, p7, p3 );
						_softBody->appendFace( p6, p3, p2 );
					}
					
					// add links for cube edges
					_softBody->appendLink( p0, p1 );
					_softBody->appendLink( p0, p3 );
					_softBody->appendLink( p0, p4 );
					// add the end, there will be one more set of links to add
					if ( k == K-1 )
					{
						_softBody->appendLink( p1, p2 );
						_softBody->appendLink( p1, p5 );
						_softBody->appendLink( p2, p6 );
					}
					if ( j == J-1 )
					{
						_softBody->appendLink( p4, p5 );
						_softBody->appendLink( p4, p7 );
						_softBody->appendLink( p5, p6 );
					}
					if ( i == I-1 )
					{
						_softBody->appendLink( p7, p3 );
						_softBody->appendLink( p3, p2 );
						_softBody->appendLink( p6, p7 );
					}
					
					// Ensure that neighboring tetras are sharing faces
					if ((i + j + k) % 2 == 1)
					{
						_softBody->appendTetra(p1,p2,p6,p3);
						_softBody->appendTetra(p3,p6,p4,p7);
						_softBody->appendTetra(p1,p4,p6,p5);
						_softBody->appendTetra(p1,p3,p4,p0);
						_softBody->appendTetra(p1,p6,p4,p3);

						_softBody->appendLink( p1, p6 );
						_softBody->appendLink( p6, p3 );
						_softBody->appendLink( p6, p4 );
						_softBody->appendLink( p1, p4 );
						_softBody->appendLink( p1, p3 );
						_softBody->appendLink( p4, p3 );
						

					}
					else
					{
						_softBody->appendTetra(p2,p0,p5,p1);
						_softBody->appendTetra(p2,p7,p0,p3);
						_softBody->appendTetra(p2,p5,p7,p6);
						_softBody->appendTetra(p0,p7,p5,p4);
						_softBody->appendTetra(p2,p0,p7,p5);
						
						_softBody->appendLink( p2, p0 );
						_softBody->appendLink( p0, p5 );
						_softBody->appendLink( p2, p7 );
						_softBody->appendLink( p7, p0 );
						_softBody->appendLink( p2, p5 );
						_softBody->appendLink( p5, p7 );
					}
				}
			}
		}
    }
	
	_bCreated = true;
	_softBody->m_materials[0]->m_kLST	=	0.9f;

	_softBody->m_cfg.piterations=2;
	_softBody->m_cfg.kDF			=0.5;
	_softBody->m_cfg.kCHR			= 1.0f;
	_softBody->m_cfg.kSHR			= 1.0f;
	_softBody->m_cfg.kSSHR_CL		=0.3;
	_softBody->m_cfg.kSS_SPLT_CL	=0.8;
	_softBody->m_cfg.kSKHR_CL		=0.1f;
	_softBody->m_cfg.kSK_SPLT_CL	=1;
	_softBody->m_cfg.collisions=	btSoftBody::fCollision::CL_SS+
	 btSoftBody::fCollision::CL_RS;
	
	
//	_softBody->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::VF_SS;
	_softBody->getCollisionShape()->setMargin(0.2);
	
	_softBody->randomizeConstraints();
	
	_softBody->setTotalMass(totalMass);
	setProperties(.4, .1);
	
	_softBody->m_cfg.citerations = 4;
	_softBody->m_cfg.diterations = 4;
	_softBody->m_cfg.piterations = 4;
	_softBody->m_cfg.viterations = 4;
	
	
	
	
	_softBody->generateClusters(128);
	

}

void ofxBulletBaseSoftShape::createFromPatch( btSoftRigidDynamicsWorld* a_world, ofVec3f topleft, ofVec3f topright, ofVec3f bottomleft, ofVec3f bottomright )
{
	_world = a_world;
	_softBody = btSoftBodyHelpers::CreatePatch( a_world->getWorldInfo(),
											   btVector3(topleft.x,topleft.y,topleft.z),
											   btVector3(topright.x,topright.y,topright.z),
											   btVector3(bottomleft.x,bottomleft.y,bottomleft.z),
											   btVector3(bottomright.x,bottomright.y,bottomright.z),
											   25,
											   25,
											   1|2|4|8,
											   true );
	_bCreated = true;

	_softBody->m_materials[0]->m_kLST	=	0.9f;

	_softBody->m_cfg.piterations=2;
	_softBody->m_cfg.kDF			=0.5;
	_softBody->m_cfg.kCHR			= 1.0f;
	_softBody->m_cfg.kSHR			= 1.0f;
	_softBody->m_cfg.kSSHR_CL		=0.3;
	_softBody->m_cfg.kSS_SPLT_CL	=0.8;
	_softBody->m_cfg.kSKHR_CL		=0.1f;
	_softBody->m_cfg.kSK_SPLT_CL	=1;
	/*_softBody->m_cfg.collisions=	btSoftBody::fCollision::CL_SS+
			btSoftBody::fCollision::CL_RS;*/
	
	 
	_softBody->m_cfg.collisions = btSoftBody::fCollision::SDF_RS | btSoftBody::fCollision::VF_SS;
	_softBody->getCollisionShape()->setMargin(0.2);

	_softBody->randomizeConstraints();

	float mass=5;
	_softBody->setTotalMass(mass);
	setProperties(.4, .1);
	
	_softBody->m_cfg.citerations = 4;
	_softBody->m_cfg.diterations = 4;
	_softBody->m_cfg.piterations = 4;
	_softBody->m_cfg.viterations = 4;
	



	_softBody->generateClusters(128);
	


}

static int nextLine(const char* buffer)
{
	int numBytesRead=0;
	
	while (*buffer != '\n')
	{
		buffer++;
		numBytesRead++;
	}
	
	
	if (buffer[0]==0x0a)
	{
		buffer++;
		numBytesRead++;
	}
	return numBytesRead;
}



btSoftBody* ofxBulletBaseSoftShape::createSoftBodyWithTetGenNodes( btSoftBodyWorldInfo& worldInfo, const char* node )
{
	btAlignedObjectArray<btVector3>	pos;
	int								nnode=0;
	int								ndims=0;
	int								nattrb=0;
	int								hasbounds=0;
	int result = sscanf(node,"%d %d %d %d",&nnode,&ndims,&nattrb,&hasbounds);
	result = sscanf(node,"%d %d %d %d",&nnode,&ndims,&nattrb,&hasbounds);
	node += nextLine(node);
	
	pos.resize(nnode);
	for(int i=0;i<pos.size();++i)
	{
		int			index=0;
		float	x,y,z;
		sscanf(node,"%d %f %f %f",&index,&x,&y,&z);
		
		node += nextLine(node);
		
		pos[index].setX(btScalar(x));
		pos[index].setY(btScalar(y));
		pos[index].setZ(btScalar(z));
	}
	btSoftBody*						psb=new btSoftBody(&worldInfo,nnode,&pos[0],0);
	printf("Nodes:  %u\r\n",psb->m_nodes.size());

	return psb;
}


void ofxBulletBaseSoftShape::appendTetGenTetras( const char* ele, bool btetralinks, btSoftBody::Material* linkMaterial ) {
	if(ele&&ele[0])
	{
		btSoftBody* psb = _softBody;
		int								ntetra=0;
		int								ncorner=0;
		int								neattrb=0;
		sscanf(ele,"%d %d %d",&ntetra,&ncorner,&neattrb);
		ele += nextLine(ele);
		
		//se>>ntetra;se>>ncorner;se>>neattrb;
		for(int i=0;i<ntetra;++i)
		{
			int			index=0;
			int			ni[4];
			
			//se>>index;
			//se>>ni[0];se>>ni[1];se>>ni[2];se>>ni[3];
			sscanf(ele,"%d %d %d %d %d",&index,&ni[0],&ni[1],&ni[2],&ni[3]);
			ele+=nextLine(ele);
			//for(int j=0;j<neattrb;++j)
			//	se>>a;
			psb->appendTetra(ni[0],ni[1],ni[2],ni[3]);
			if(btetralinks)
			{
				//if ( ofRandomuf()>0.5f )
				{
					psb->appendLink(ni[0],ni[1],linkMaterial,true);
					psb->appendLink(ni[1],ni[2],linkMaterial,true);
					psb->appendLink(ni[2],ni[0],linkMaterial,true);
				}
				//else
				{
					psb->appendLink(ni[0],ni[3],linkMaterial,true);
					psb->appendLink(ni[1],ni[3],linkMaterial,true);
					psb->appendLink(ni[2],ni[3],linkMaterial,true);
				}
			}
		}
	}

	printf("Tetras: %u\r\n",_softBody->m_tetras.size());

}


void ofxBulletBaseSoftShape::appendTetGenFaces( const char* face, bool makeFaceLinks, btSoftBody::Material* linkMaterial ) {
	// ok, it seems we have to manually add faces
	if ( face && face[0] ) {
		stringstream ss( face );
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
			//ofLogNotice("ofxBullBaseSoftShape") << faceIndex << " " << n0 << " " << n1 << " " << n2;
			_softBody->appendFace(n2, n1, n0);
			// randomly skip two links
			//int skipLink = ofRandom(2.99999999999);
			if ( makeFaceLinks ) {
				//if ( skipLink == 0 )
					_softBody->appendLink( n0, n1, linkMaterial );
				//if ( skipLink == 1 )
					_softBody->appendLink( n1, n2, linkMaterial );
				//if ( skipLink == 2 )
					_softBody->appendLink( n2, n0, linkMaterial );
			}
			externalNodes.insert(n0);
			externalNodes.insert(n1);
			externalNodes.insert(n2);
		}
	}
	
}



void ofxBulletBaseSoftShape::createFromTetraBuffer( btSoftRigidDynamicsWorld* a_world, ofBuffer& eleFile, ofBuffer& faceFile, ofBuffer& nodeFile, btTransform a_bt_tr,
												   float a_mass, float scale, float springStrength, float bendingConstraintsSpringStrength, float borderSpringStrength ) {
	
	_world = a_world;

	bool makeFaceLinks = true;
	bool makeTetraLinks = true;
	bool makeFacesFromTetras = false; // has no effect anyway
	

	// create
	_softBody = createSoftBodyWithTetGenNodes( a_world->getWorldInfo(), nodeFile.getBinaryBuffer() );
	_softBody->scale( btVector3(scale, scale, scale) );
	_softBody->transform( a_bt_tr );
	_bCreated = true;
	


	// setup materials
	btSoftBody::Material* tetraMaterial = _softBody->m_materials[0];
	btSoftBody::Material* faceMaterial = _softBody->appendMaterial();
	btSoftBody::Material* bendingConstraintsMaterial = _softBody->appendMaterial();
	int borderMaterialIndex = 1;
	*faceMaterial = *tetraMaterial;
	tetraMaterial->m_kLST = springStrength;
	tetraMaterial->m_kVST = 1.0f;
	if ( borderSpringStrength>=0 )
		faceMaterial->m_kLST = borderSpringStrength;
	else
		faceMaterial->m_kLST = springStrength;
	if ( bendingConstraintsSpringStrength>= 0 )
		bendingConstraintsMaterial->m_kLST = bendingConstraintsSpringStrength;
	else
		bendingConstraintsMaterial->m_kLST = springStrength;

	
	
	// add faces
	appendTetGenFaces( faceFile.getBinaryBuffer(), makeFaceLinks, faceMaterial );
	// create bending constraints for face links
	_softBody->generateBendingConstraints(3, bendingConstraintsMaterial);
	
	
	// add tetras
	appendTetGenTetras( eleFile.getBinaryBuffer(), makeTetraLinks, tetraMaterial );
	
	
	// config
	
	//_softBody->m_cfg.collisions |= btSoftBody::fCollision::SDF_RS;
	
	_softBody->m_cfg.collisions	= btSoftBody::fCollision::CL_RS;
	//_softBody->m_cfg.collisions |= btSoftBody::fCollision::CL_SELF;
//	_softBody->m_cfg.collisions = 0;
	_softBody->randomizeConstraints();
	
	_softBody->m_cfg.citerations=4;
	_softBody->m_cfg.piterations=8;
	_softBody->m_cfg.viterations=4;
	_softBody->m_cfg.diterations=2;
	_softBody->m_cfg.kAHR = 1.0f; // anchors hardness
	_softBody->m_cfg.kDF			=0.2; // dynamic friction 0..1
	_softBody->m_cfg.kVC = 1.0f; // volume conservation 0..inf
	_softBody->m_cfg.kSHR = 1.0f; // soft contacts hardness 0..1
	_softBody->m_cfg.kCHR = 1.0f; // rigid contacts hardness 0..1
	_softBody->m_cfg.kSRHR_CL		= 0.1f; // soft vs rigid hardness
	_softBody->m_cfg.kSKHR_CL		= 1.0f; // soft vs kinetic hardness
	_softBody->m_cfg.kSSHR_CL		= 0.5f; // soft vs soft hardness
	_softBody->m_cfg.kSR_SPLT_CL	= 0.5f; // soft vs rigid impulse split
	_softBody->m_cfg.kSK_SPLT_CL	= 0.5f; // soft vs kinetic impulse split
	_softBody->m_cfg.kSS_SPLT_CL	= 0.5f; // soft vs soft impulse split
	
	
	_softBody->m_cfg.kTetraPressure = 0;
	_softBody->m_cfg.kHydrostatic = 0;
	

	assert(_softBody->getCollisionShape());
	_softBody->getCollisionShape()->setMargin(0.1);
	
	_softBody->setPose( true, false );

	
	_softBody->randomizeConstraints();


	_softBody->generateClusters(128);
	
	setProperties(.9, .1);
	
	
	//_softBody->m_cfg.collisions	=	btSoftBody::fCollision::CL_SS | btSoftBody::fCollision::CL_RS;

	//_softBody->m_cfg.kKHR = 1.0f; // penetration with kinetic
	//_softBody->m_cfg.piterations = 2;
	_softBody->setTotalMass( a_mass );
}




void ofxBulletBaseSoftShape::createFromOfMesh( btSoftRigidDynamicsWorld* a_world, const ofMesh& mesh, btTransform a_bt_tr, float a_mass, float scale ){
	
	if ( mesh.getMode() != OF_PRIMITIVE_TRIANGLES ) {
		ofLogError("ofxBulletBaseSoftShape") << "create needs mesh with mode OF_PRIMITIVE_TRIANGLES";
		return;
	}

	if(a_world == NULL) {
		ofLog(OF_LOG_ERROR, "ofxBulletBaseSoftShape :: create : a_world param is NULL");
		return;
	}
	_world			= a_world;
	
	_bCreated		= true;
	
	 
	vector<ofVec3f> vertices = mesh.getVertices();

	
	vector<int> triangles;
	for ( int i=0; i<mesh.getNumIndices(); i++ )
		triangles.push_back( mesh.getIndex(i) );
	
	int numTriangles = triangles.size()/3;
	_softBody		= btSoftBodyHelpers::CreateFromTriMesh( a_world->getWorldInfo(), &(vertices[0].x), &triangles[0], numTriangles );
	_softBody->scale( btVector3(scale, scale, scale));
	setProperties(.9, .1);
	
//	_softBody->m_materials[0]->m_kLST = 0.3f;
	_softBody->generateBendingConstraints(3);
	//_softBody->generateClusters(0);
	_softBody->m_cfg.kKHR = 1.0f; // penetration with kinetic
	_softBody->m_cfg.kCHR = 0.8; // penetration
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



void ofxBulletBaseSoftShape::moveNode( int nodeIndex, ofVec3f pos, float timeStep ) {
	btVector3 delta = btVector3( pos.x, pos.y, pos.z ) - _softBody->m_nodes[nodeIndex].m_x;
	_softBody->m_nodes[nodeIndex].m_v += delta/timeStep;
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
float ofxBulletBaseSoftShape::getMass() const {
	return _softBody->getTotalMass();
}


void ofxBulletBaseSoftShape::setMass(float mass) {
	_softBody->setTotalMass( mass, false, true );
}


//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getRestitution() const {
	return (float)_softBody->getRestitution();
}

//--------------------------------------------------------------
float ofxBulletBaseSoftShape::getFriction() const {
	return _softBody->getFriction();
}




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


void ofxBulletBaseSoftShape::drawLinksWithMaterial(int materialIndex)
{
	ofMesh mesh;
	for(int i=0;i<_softBody->m_links.size();++i)
	{
		const btSoftBody::Link&	l=_softBody->m_links[i];
		if ( l.m_material == _softBody->m_materials[materialIndex] ) {
			const btVector3& a = l.m_n[0]->m_x;
			const btVector3& b = l.m_n[1]->m_x;
			ofVec3f aOf( a.getX(), a.getY(), a.getZ() );
			ofVec3f bOf( b.getX(), b.getY(), b.getZ() );
			mesh.addVertex(aOf);
			mesh.addVertex(bOf);
		}
	}
	mesh.setMode( OF_PRIMITIVE_LINES );
	mesh.drawWireframe();
}



void ofxBulletBaseSoftShape::draw(){
	drawFaces();
	ofMesh mesh;
	
	mesh.clear();
	for ( int i=0; i<_softBody->m_nodes.size();++i )
	{
		const btSoftBody::Node& n = _softBody->m_nodes[i];
		mesh.addVertex( ofVec3f( n.m_x.getX(), n.m_x.getY(), n.m_x.getZ() ) );
	}
	mesh.drawVertices();
	
	/*
	for ( int i=0; i<_softBody->m_materials.size(); i++ )
		drawLinksWithMaterial(i);*/
}

void ofxBulletBaseSoftShape::drawFaces(){

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
	
}



void ofxBulletBaseSoftShape::setPressure( float pressure ) // -inf..inf
{
	_softBody->m_cfg.kPR = pressure;
}

float ofxBulletBaseSoftShape::getPressure() {
	return _softBody->m_cfg.kPR;
}

void ofxBulletBaseSoftShape::setVolumeConservationCoefficient( float coeff )
{
	_softBody->m_cfg.kVC = coeff;
}

float ofxBulletBaseSoftShape::getVolumeConservationCoefficient()
{
	return _softBody->m_cfg.kVC;
}


void ofxBulletBaseSoftShape::setDamping( float damping ) // 0..1, 1 = fully damped
{
	_softBody->m_cfg.kDP = damping;
}

float ofxBulletBaseSoftShape::getDamping() {
	return _softBody->m_cfg.kDP;
}

void ofxBulletBaseSoftShape::setTetraPressure( float p ) 
{
	_softBody->m_cfg.kTetraPressure = p;
}

float ofxBulletBaseSoftShape::getTetraPressure() {
	return _softBody->m_cfg.kTetraPressure;
}


vector<ofVec3f> ofxBulletBaseSoftShape::getNodeLocations()
{
	vector<ofVec3f> output;
	for ( int i=0; i<_softBody->m_nodes.size(); i++ ) {
		btVector3 pos =  _softBody->m_nodes[i].m_x;
		output.push_back( ofVec3f(pos.getX(), pos.getY(), pos.getZ()) );
	}
	return output;
}

void ofxBulletBaseSoftShape::addLink( int index0, int index1, int materialIndex, bool suppressGenerateClusters, bool replaceMaterial )
{
	int existingIndex = getIndexOfLinkBetween(index0, index1);
	if ( existingIndex == -1 ) {
		_softBody->appendLink( index0, index1, _softBody->m_materials[materialIndex] );
		if ( !suppressGenerateClusters )
			generateClusters();
	} else if ( replaceMaterial ) {
		getLink( existingIndex ).m_material = _softBody->m_materials[materialIndex];
	}
		
}

void ofxBulletBaseSoftShape::addNode( ofVec3f pos, float mass ) {
	_softBody->appendNode( btVector3(pos.x,pos.y,pos.z), mass );
}

int ofxBulletBaseSoftShape::getIndexOfLinkBetween(int node0, int node1) {
	if ( ! _softBody->checkLink(node0, node1) )
		return -1;
	
	const btSoftBody::Node* n[]={&getNode(node0),&getNode(node1)};
	for(int i=0,ni=getNumLinks();i<ni;++i)
	{
		const btSoftBody::Link&	l=getLink(i);
		if(	(l.m_n[0]==n[0]&&l.m_n[1]==n[1])||
		   (l.m_n[0]==n[1]&&l.m_n[1]==n[0]))
		{
			return(i);
		}
	}
	return(-1);
	
}

int ofxBulletBaseSoftShape::getMaterialIndexForLink(int linkIndex) {
	btSoftBody::Link& l = getLink(linkIndex);
	for ( int i=0; i<_softBody->m_materials.size(); i++ ) {
		if ( l.m_material == _softBody->m_materials[i] )
			return i;
	}
	assert(false);
	return -1;
}

int ofxBulletBaseSoftShape::getIndexOfNode(btSoftBody::Node *n) {
	int index = n - &(_softBody->m_nodes[0]);
	if ( index <0 || index >= _softBody->m_nodes.size() )
		return -1;
	else
		return index;
}

set<int> ofxBulletBaseSoftShape::getAllLinksTouching( int nodeIndex ) {
	set<int> results;
	btSoftBody::Node* n = &getNode(nodeIndex);
	for ( int i=0; i<_softBody->m_links.size(); i++ ) {
		btSoftBody::Link& l = getLink(i);
		if ( l.m_n[0] == n )
			results.insert(i);
		else if ( l.m_n[1] == n )
			results.insert(i);
		
	}
	return results;
}

set<int> ofxBulletBaseSoftShape::getAllNeighboursOf( int nodeIndex ){
	set<int> links = getAllLinksTouching( nodeIndex );
	btSoftBody::Node* n = &getNode(nodeIndex);
	set<int> results;
	for ( set<int>::iterator it = links.begin(); it != links.end(); ++it ) {
		btSoftBody::Link& l = getLink(*it);
		if ( l.m_n[0] == n )
			results.insert(getIndexOfNode(l.m_n[1]));
		else if ( l.m_n[1] == n )
			results.insert(getIndexOfNode(l.m_n[0]));
	}
	return results;
}

float ofxBulletBaseSoftShape::getRestVolume() const {
	if ( _softBody->m_pose.m_volume )
		return _softBody->m_pose.m_volume;
	else
		return 0;
}

float ofxBulletBaseSoftShape::getVolume() const {
	return _softBody->getVolume();
}

float ofxBulletBaseSoftShape::calculateVolumeOfTetras() const {
	double volume = 0;
	for ( int i=0; i<_softBody->m_tetras.size(); i++ ) {
		btSoftBody::Tetra& t = _softBody->m_tetras[i];
		const btVector3& a = t.m_n[0]->m_x;
		const btVector3& b = t.m_n[1]->m_x;
		const btVector3& c = t.m_n[2]->m_x;
		const btVector3& d = t.m_n[3]->m_x;
		double v = ( (a-b).dot((b-d).cross(c-d))) / 6.0;
		volume += v;
	}
	return volume;
}

void ofxBulletBaseSoftShape::anchorNode( int nodeIndex, btRigidBody* anchorBody, ofVec3f offs )
{
	btVector3 offset = btVector3(offs.x, offs.y, offs.z);
	_softBody->appendAnchor( nodeIndex, anchorBody, offset );
}

