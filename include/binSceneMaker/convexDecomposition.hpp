#pragma once

// This hpp comes from bullet demo's source.
// Thanks.
// https://github.com/kripken/bullet/blob/master/Demos/ConvexDecompositionDemo/ConvexDecompositionDemo.cpp

#include <btBulletDynamicsCommon.h>
#include <LinearMath/btQuickprof.h>
#include <LinearMath/btGeometryUtil.h>
#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionDispatch/btCompoundCollisionAlgorithm.h> //for callback
#include <ConvexDecomposition/ConvexDecomposition.h>
#include <HACD/hacdHACD.h>

bool MyCompoundChildShapeCallback(
	const btCollisionShape* pShape0,
	const btCollisionShape* pShape1);

bool MyContactCallback(
	btManifoldPoint& cp,
	const btCollisionObjectWrapper* colObj0Wrap,
	int partId0,
	int index0,
	const btCollisionObjectWrapper* colObj1Wrap,
	int partId1,
	int index1);

class MyConvexDecomposition : public ConvexDecomposition::ConvexDecompInterface
{
public:
	virtual void ConvexDecompResult(ConvexDecomposition::ConvexResult &result);

	int mBaseCount = 0;
	int mHullCount = 0;
	btAlignedObjectArray<btConvexHullShape*> m_convexShapes;
	btAlignedObjectArray<btVector3> m_convexCentroids;
	btAlignedObjectArray<btTriangleMesh*> m_trimeshes;
	btAlignedObjectArray<btConvexHullShape*> m_collisionShapes;
	btVector3 centroid;
};
