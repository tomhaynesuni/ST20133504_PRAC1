#ifndef PLAYER_H_
#define PLAYER_H_

/* Ogre3d Graphics*/
#include "Ogre.h"

/* Bullet3 Physics */
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"

using namespace Ogre;

/** Example player class.
* Written to illistrate the connection of Ogre/Bullet.
* Essentially just a wrapper around the cube object setup code.
*/
class Player
{
private:
	SceneNode* boxSceneNode;     /**< Scene graph node */
	Entity* box;                 /**< Mesh entity */
	Vector3 meshBoundingBox;     /**< Size of the bounding mesh from ogre */

	btCollisionShape* colShape;  /**< Collision shape, describes the collision boundary */
	btRigidBody* body;           /**< Rigid Body */
	btDiscreteDynamicsWorld* dynamicsWorld;  /**< physics/collision world */

	float forwardForce; /**< Force of the engine/thrusters moving the player forward */
	float turningForce; /**< Force exerted by the turning of the wheels or thrusters */
	btScalar linearDamping; /**< Damping force on the linear motion of the body, kind of air/friction */
	btScalar angularDamping; /**< Damping force on the angular motion of the body, kind of air/friction */

public:
	Player();
	~Player();

	/**
	* Creates the mesh.
	* @param scnMgr the Ogre SceneManager.
	*/
	void createMesh(SceneManager* scnMgr);
	/**
	* Creates a new child of the given parent node, adds the mesh to it.
	* @param parent, the parent (in the scene graph) of the node the player will be attatched to.
	*/
	void attachToNode(SceneNode* parent);


	/**
	* Sets the scale.
	* @param x, scale on the x axis.
	* @param y, scale on the y axis.
	* @param z, scale on the z axis.
	*/
	void setScale(float x, float y, float z);
	/**
	* Sets the orientation.
	* @param axis, vector about which the orientation takes place.
	* @param angle, angle (in radians).
	*/
	void setRotation(Vector3 axis, Radian angle);

	/**
	* Sets the position.
	* @param x, position on the x axis.
	* @param y, position on the y axis.
	* @param z, position on the z axis.
	*/  void setPosition(float x, float y, float z);

	/**
	* Fudge to get the bouning box from Ogre3d, at a it might work for other shapes.
	*/
	void boundingBoxFromOgre();

	/**
	* Creates a new ridgid body of the given mass.
	* @param mass
	*/
	void createRigidBody(float mass);
	/**
	* Add this collision shape to the collision shapes list
	* @param collisionShaps, the list of collision shapes (shared with the physics world).
	*/
	void addToCollisionShapes(btAlignedObjectArray<btCollisionShape*>& collisionShapes);
	/**
	*  Add this rigid body to the dynamicsWorld.
	* @param dynamicsWorld, the wrold we're going to add ourselves to.
	*/
	void addToDynamicsWorld(btDiscreteDynamicsWorld* dynamicsWorld);

	/**
	* What on Earth is this for!? Can't change the mass of a rigid body.
	* @param mass
	*/
	void setMass(float mass);

	/**
	* Update, um ... makes coffee.
	*/
	void update();

	/**
	* Moves the player forward with maximum acceleration.
	*/
	void forward();

	/**
	* Turn the player to the right, use a point on the front of the cube
	* this will hopefully translate to more vehicle like movement.
	*/
	void turnRight();

	/**
	* Turn the player to the right, use a torque on the center point
	* this will give the player 'midtown madness' or thruster like steering.
	*/
	void spinRight();


};


#endif
