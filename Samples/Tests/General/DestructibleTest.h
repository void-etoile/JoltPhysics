// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>
#include <mutex>

/// Demonstrates frame-and-infill destructible environments.
/// Structural chunks (columns, beams, roof slabs) take damage from contact impulses
/// detected via ContactListener. When accumulated damage exceeds the break threshold
/// the chunk fractures into Voronoi debris. A BFS support-graph pass propagates
/// structural collapse to unsupported elements. Infill panels are lighter and
/// break from much smaller impacts; they also break when pulled >5 cm from their
/// attachment points (gap check). Adjust the sliders to tune both thresholds.
class DestructibleTest : public Test, public ContactListener
{
public:
	JPH_DECLARE_RTTI_VIRTUAL(JPH_NO_EXPORT, DestructibleTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "A city block with a main demo wall, houses, and multi-story apartment buildings. "
			   "Panels break from low-energy impacts; frame elements require much more. "
			   "Press Enter to fire projectiles. Space to drag bodies. "
			   "Lower Frame Break Force to collapse whole structures.";
	}

	// See: Test
	virtual void			Initialize() override;
	virtual void			ProcessInput(const ProcessInputParams &inParams) override;
	virtual void			PrePhysicsUpdate(const PreUpdateParams &inParams) override;

	// Optional settings menu
	virtual bool			HasSettingsMenu() const override							{ return true; }
	virtual void			CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu) override;

	virtual bool			IsDeterministic() const override							{ return false; }

	// Live status overlay
	virtual String			GetStatusString() const override;

	// Saving / restoring input state for replay
	virtual void			SaveInputState(StateRecorder &inStream) const override;
	virtual void			RestoreInputState(StateRecorder &inStream) override;

	// ContactListener — accumulate per-body impact impulses on the physics thread
	virtual ValidateResult	OnContactValidate(const Body &, const Body &, RVec3Arg, const CollideShapeResult &) override { return ValidateResult::AcceptAllContactsForThisBodyPair; }
	virtual void			OnContactAdded(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override;
	virtual void			OnContactPersisted(const Body &, const Body &, const ContactManifold &, ContactSettings &) override {}
	virtual void			OnContactRemoved(const SubShapeIDPair &) override {}

private:
	struct FractureInfo
	{
		Array<RefConst<Shape>>	mShapes;		// one convex hull per Voronoi cell
		Array<Vec3>				mLocalCenters;	// centroid of each cell in panel local space
		bool					mIsFrame = false; // true → break when frame constraints gone; false → panel constraints
	};

	void					FireProjectile(RVec3Arg inPos, Vec3Arg inDirection);
	void					BuildMainWall();
	void					BuildHouse(RVec3Arg inCenter);
	void					BuildApartment(RVec3Arg inCenter, int inNumFloors, float inHalfW, float inHalfD);
	void					BuildTower(RVec3Arg inCenter, int inNumFloors, float inRadius, int inNumSides);
	void					BuildHighrise(RVec3Arg inCenter, int inNumFloors, int inFloorsPerSeg, float inHalfW, float inHalfD);
	void					CheckStructuralIntegrity();
	void					CheckGravitationalMoment();
	void					CheckSupportStability();
	void					SpawnFracture(BodyID inPanelID);

	struct ProjectileRecord { BodyID mID; float mLifeRemaining; };

	Array<Ref<FixedConstraint>>			mPanelConstraints;	// infill panels
	Array<Ref<SixDOFConstraint>>		mFrameConstraints;	// structural frame
	UnorderedMap<BodyID, FractureInfo>	mFractureData;		// registered chunks → fracture geometry
	Array<BodyID>						mShardBodies;		// live shard bodies, cleaned up when sleeping
	Array<ProjectileRecord>				mProjectiles;		// live projectiles with remaining lifetime

	// Adjacency index: O(degree) constraint lookup and removal.
	UnorderedMap<BodyID, int>							mFrameConnCount;
	UnorderedMap<BodyID, int>							mPanelConnCount;
	UnorderedMap<BodyID, Array<Ref<SixDOFConstraint>>>	mFrameAdj;
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>>	mPanelAdj;
	UnorderedMap<SixDOFConstraint *, int>				mFrameIdx;
	UnorderedMap<FixedConstraint *, int>				mPanelIdx;

	// Rest-pose relative rotation per frame constraint (for deformation angle measurement).
	UnorderedMap<SixDOFConstraint *, Quat>				mConstraintRestRot;

	// Per-chunk accumulated damage (N·s). Filled from mPendingDamage each frame.
	UnorderedMap<BodyID, float>							mChunkDamage;

	// Thread-safe staging buffers: OnContactAdded (physics thread) pushes here;
	// PrePhysicsUpdate (main thread) drains them under mDamageMutex.
	std::mutex										mDamageMutex;
	Array<std::pair<BodyID, float>>					mPendingDamage;
	Array<BodyID>									mHitProjectiles;  // projectiles that struck something this step

	// Fast O(1) lookup so OnContactAdded can identify projectiles without scanning mProjectiles.
	// Written only on the main thread (FireProjectile / expire loop); safe to read from physics thread.
	UnorderedSet<BodyID>							mProjectileSet;

	// Create a frame (SixDOFConstraint) or panel (FixedConstraint) between inA and inB, register everywhere.
	void					TrackConstraint(bool inIsFrame, Body *inA, Body *inB);

	// Swap-and-pop removal of the constraint at inPos; updates all indices.
	void					UntrackConstraint(bool inIsFrame, int inPos);

	uint32					mNextBuildingGroupID = 1;	// unique per-building so inter-building collision passes GroupFilterTable
	int						mInitialPanelCount = 0;
	int						mInitialFrameCount = 0;
	float					mLastBreakCheckUs = 0.0f;
	int						mBreakLogCount = 0;
	FILE *					mLogFile = nullptr;

	bool					mFire = false;
	bool					mWasFire = false;

	static float			sPanelBreakForce;
	static float			sFrameBreakForce;
	static float			sFrameBreakMoment;      // gravitational bending moment threshold (N·m)
	static float			sFrameBreakAxial;       // gravitational axial force threshold (N) for bridge constraints
	static float			sFrameBendThreshold;    // max deformation angle (rad) before a frame constraint breaks
	static float			sFrameSpringStiffness;  // N·m/rad — rotational stiffness of frame joints
	static float			sFrameSpringDamping;    // N·m·s/rad — rotational damping of frame joints
	static float			sFrameSwayBreakRate;    // rad/s — relative angular velocity at which a joint snaps (prevents whip oscillation)
};
