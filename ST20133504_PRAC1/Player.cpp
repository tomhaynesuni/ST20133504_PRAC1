#include "Player.h"

Player::Player()
{
    boxSceneNode = nullptr;
    box = nullptr;
    Vector3 meshBoundingBox(0.0f, 0.0f, 0.0f);

    colShape = nullptr;
    dynamicsWorld = nullptr;

    /* Note: These are hardcoded in player.  Should probably be read in from a
    * config file or similar.
    */

    forwardForce = 100.0f;
    turningForce = 20.0f;
    linearDamping = 0.2f;
    angularDamping = 0.8f;
}

Player::~Player()
{

}

void Player::createMesh(SceneManager* scnMgr)
{
    box = scnMgr->createEntity("cube.mesh");
}

void Player::attachToNode(SceneNode* parent)
{
    boxSceneNode = parent->createChildSceneNode();
    boxSceneNode->attachObject(box);
    boxSceneNode->setScale(1.0f, 1.0f, 1.0f);
    boundingBoxFromOgre();
}

void Player::setScale(float x, float y, float z)
{
    boxSceneNode->setScale(x, y, z);
}


void Player::setRotation(Vector3 axis, Radian rads)
{
    //quat from axis angle
    Quaternion quat(rads, axis);
    boxSceneNode->setOrientation(quat);
}

void Player::setPosition(float x, float y, float z)
{
    boxSceneNode->setPosition(x, y, z);
}

void Player::boundingBoxFromOgre()
{
    //get bounding box here.
    boxSceneNode->_updateBounds();
    const AxisAlignedBox& b = boxSceneNode->_getWorldAABB();
    Vector3 temp(b.getSize());
    meshBoundingBox = temp;
}

void Player::createRigidBody(float bodyMass)
{
    colShape = new btBoxShape(btVector3(meshBoundingBox.x / 2.0f, meshBoundingBox.y / 2.0f, meshBoundingBox.z / 2.0f));

    /// Create Dynamic Objects
    btTransform startTransform;
    startTransform.setIdentity();

    Quaternion quat2 = boxSceneNode->_getDerivedOrientation();
    startTransform.setRotation(btQuaternion(quat2.x, quat2.y, quat2.z, quat2.w));

    Vector3 pos = boxSceneNode->_getDerivedPosition();
    startTransform.setOrigin(btVector3(pos.x, pos.y, pos.z));

    btScalar mass(bodyMass);

    //rigidbody is dynamic if and only if mass is non zero, otherwise static
    bool isDynamic = (mass != 0.f);

    btVector3 localInertia(0, 0, 0);
    if (isDynamic)
    {
        // Debugging
        //std::cout << "I see the cube is dynamic" << std::endl;
        colShape->calculateLocalInertia(mass, localInertia);
    }

    //using motionstate is recommended, it provides interpolation capabilities, and only synchronizes 'active' objects
    btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
    btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, myMotionState, colShape, localInertia);
    body = new btRigidBody(rbInfo);

    //Set the linear and angular damping
    //I'm using this to bring the object to rest when moving.
    //An alternative would be to use friciton for the collison.
    //No good for hovering stuff though.
    body->setDamping(linearDamping, angularDamping);

    //Set the user pointer to this object.
    body->setUserPointer((void*)this);
}

void Player::addToCollisionShapes(btAlignedObjectArray<btCollisionShape*>& collisionShapes)
{
    collisionShapes.push_back(colShape);
}

void Player::addToDynamicsWorld(btDiscreteDynamicsWorld* dynamicsWorld)
{
    this->dynamicsWorld = dynamicsWorld;
    dynamicsWorld->addRigidBody(body);
}

void Player::update()
{
    btTransform trans;

    if (body && body->getMotionState())
    {
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        boxSceneNode->setPosition(Ogre::Vector3(trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ()));
        boxSceneNode->setOrientation(Ogre::Quaternion(orientation.getW(), orientation.getX(), orientation.getY(), orientation.getZ()));
    }
}

 void Player::forward()
{
    //Create a vector in local coordinates
    //pointing down z.
    btVector3 fwd(0.0f, 0.0f, forwardForce);
    btVector3 push;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //get the orientation of the rigid body in world space.
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //rotate the local force, into the global space.
        //i.e. push in down the local z.
        push = quatRotate(orientation, fwd);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        //apply a force to the center of the body
        body->applyCentralForce(push);
    }
} 

void Player::turnRight()
{
    //Apply a turning force to the front of the body.
    btVector3 right(turningForce, 0.0f, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //again get the orientation of the body.
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //get the position of the body, so we can identify the
        //front and push it.
        btVector3 front(trans.getOrigin());

        //use original bounding mesh to get the front center
        front += btVector3(0.0f, 0.0f, meshBoundingBox.z / 2);

        //orientated the local force into world space.
        turn = quatRotate(orientation, right);

        //took this out, can't turn if your not moving.
        //body->activate();

        //better - only turn if we're moving.
        //not ideal, if sliding sideways will keep turning.
        if (body->getLinearVelocity().length() > 0.0f)
            body->applyForce(turn, front);
    }
}


void Player::spinRight()
{
    //Apply a turning force to the front of the body.
    //this is an axis (around which to turn)
    //lenght of the vector is the magnitue of the torque.
    btVector3 right(0.0f, 100.0f, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //again get the orientation of the body.
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //orientated the local force into world space.
        turn = quatRotate(orientation, right);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        body->applyTorque(right);

    }
}

/*
void Player::spinLeft()
{
    //Apply a turning force to the front of the body.
    //this is an axis (around which to turn)
    //lenght of the vector is the magnitue of the torque.
    btVector3 left(0.0f, 100.0f, 0.0f);
    btVector3 turn;

    btTransform trans;

    if (body && body->getMotionState())
    {
        //again get the orientation of the body.
        body->getMotionState()->getWorldTransform(trans);
        btQuaternion orientation = trans.getRotation();

        //orientated the local force into world space.
        turn = quatRotate(orientation, left);

        //activate the body, this is essential if the body
        //has gone to sleep (i.e. stopped moving/colliding).
        body->activate();

        body->applyTorque(left);

    }
}

*/
