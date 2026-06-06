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
			   "Press Enter to fire projectiles. Space to drag bodies. Press K to destroy every "
			   "building's foundation at once (stress test). "
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
	virtual void			OnContactPersisted(const Body &inBody1, const Body &inBody2, const ContactManifold &inManifold, ContactSettings &ioSettings) override;
	virtual void			OnContactRemoved(const SubShapeIDPair &) override {}

private:
	struct FractureInfo
	{
		Array<RefConst<Shape>>	mShapes;		// one convex hull per Voronoi cell
		Array<Vec3>				mLocalCenters;	// centroid of each cell in panel local space
		bool					mIsFrame = false; // true → break when frame constraints gone; false → panel constraints
		bool					mIsFloor = false; // true → use sFloorBreakForce instead of sPanelBreakForce
	};

	void					FireProjectile(RVec3Arg inPos, Vec3Arg inDirection);
	void					BuildMainWall();
	void					BuildHouse(RVec3Arg inCenter);
	void					BuildApartment(RVec3Arg inCenter, int inNumFloors, float inHalfW, float inHalfD);
	void					BuildTower(RVec3Arg inCenter, int inNumFloors, float inRadius, int inNumSides);
	void					BuildHighrise(RVec3Arg inCenter, int inNumFloors, int inFloorsPerSeg, float inHalfW, float inHalfD);
	void					BuildEiffelTower(RVec3Arg inCenter);
	void					BuildCastle(RVec3Arg inCenter, float inHalfSize);
	// inAdj is a frame-graph adjacency snapshot built by the caller and shared across these passes
	// (rebuilt only when a pass actually breaks a joint), avoiding a whole-scene rebuild in each.
	void					CheckStructuralIntegrity(const UnorderedSet<BodyID> &inScope, const UnorderedMap<BodyID, Array<int>> &inAdj);
	void					CheckGravitationalMoment(const UnorderedSet<BodyID> &inScope, const UnorderedMap<BodyID, Array<int>> &inAdj);
	void					CheckSupportStability(const UnorderedSet<BodyID> &inScope, const UnorderedMap<BodyID, Array<int>> &inAdj);
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

	// Accumulated time (s) each frame joint has spent past the plastic-yield angle.
	// A joint stuck in sustained yield (e.g. an overloaded cantilever at equilibrium) fractures
	// once this exceeds sFrameYieldTimeLimit, even if it never reaches the hard bend threshold.
	UnorderedMap<SixDOFConstraint *, float>				mJointYieldTime;

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

	// Bodies whose frame constraints were broken since the last cascade run.
	// Cleared just before the structural cascade so the cascade's own breaks feed into the next frame.
	UnorderedSet<BodyID> mRecentlyBrokenFrameBodies;

	// Bodies whose connection count changed (any constraint untracked) and so might now be
	// zero-connection. Lets the zero-connection fracture pass check only these instead of
	// scanning the whole scene every frame. Over-budget entries persist for the next frame.
	UnorderedSet<BodyID> mConnDirty;

	// Building-scoped frame-joint index for the deformation / isolation passes. Each frame
	// constraint is tagged with a STABLE structural building id (the gid its Build* assigned at
	// construction — never changed by the collision-group regrouping, which allocates fresh gids).
	// Once a building loses a joint it is added to mDamagedBuildings, and those passes then iterate
	// only the joints of damaged buildings instead of the whole scene — so an intact, sleeping
	// building costs nothing, which cuts the per-frame peak rather than just the average.
	uint32												mCurrentBuildingGroup = 0;	// set by each Build* before tracking
	UnorderedMap<uint32, Array<Ref<SixDOFConstraint>>>	mBuildingFrameConstraints;	// stable building id → its frame joints
	UnorderedMap<SixDOFConstraint *, uint32>			mFrameConstraintBuilding;	// frame joint → its stable building id
	UnorderedSet<uint32>								mDamagedBuildings;			// buildings that have lost ≥1 joint

	// Bodies the unground pass has already given a fresh collision group. Ungrounding is monotonic
	// (a freed body never re-grounds), so each body is regrouped exactly once — without this the
	// pass would re-write the collision group of every ungrounded body every frame.
	UnorderedSet<BodyID>								mRegroupedBodies;

	// Frame-graph snapshot (adjacency + ground-reachable set) persisted ACROSS frames and rebuilt
	// only when the frame-constraint count changes (i.e. only on frames where a joint broke). The
	// graph is otherwise static, so on the common "leaning under load" frame this is reused for
	// free, removing the per-frame O(all-joints) adjacency build + grounded BFS from the hot path.
	UnorderedMap<BodyID, Array<int>>					mSnapFrameAdj;
	UnorderedSet<BodyID>								mSnapGrounded;
	int													mSnapCount = -1;	// constraint count when the snapshot was built; -1 = invalid
	void					EnsureFrameSnapshot();

	// Create a frame (SixDOFConstraint with spring motors) or panel (FixedConstraint) between inA and inB.
	void					TrackConstraint(bool inIsFrame, Body *inA, Body *inB);

	UnorderedMap<BodyID, Array<int>> BuildFrameAdjacency() const;

	// BFS from all static (ground-anchor) bodies through the frame graph in inAdj; fills outGrounded
	// with every body that can reach the ground. Shared by the deformation and unground passes.
	void					ComputeGroundedSet(const UnorderedMap<BodyID, Array<int>> &inAdj, UnorderedSet<BodyID> &outGrounded);

	// Expand inSeeds to all bodies in the same frame-constraint components, writing results into outScope.
	void ExpandToComponents(const UnorderedSet<BodyID> &inSeeds, const UnorderedMap<BodyID, Array<int>> &inAdj, UnorderedSet<BodyID> &outScope) const;

	// Swap-and-pop removal of the constraint at inPos; updates all indices.
	void					UntrackConstraint(bool inIsFrame, int inPos);

	uint32					mNextBuildingGroupID = 1;	// unique per-building so inter-building collision passes GroupFilterTable
	int						mInitialPanelCount = 0;
	int						mInitialFrameCount = 0;
	float					mLastBreakCheckUs = 0.0f;

	bool					mFire = false;
	bool					mWasFire = false;
	bool					mDestroyBases = false;	// stress test: sever every building's foundation joints at once
	bool					mWasDestroyBases = false;

	static float			sPanelBreakForce;
	static float			sFloorBreakForce;
	static float			sFrameBreakForce;
	static float			sFrameBreakMoment;      // gravitational bending moment threshold (N·m)
	static float			sFrameBreakAxial;       // gravitational axial force threshold (N) for bridge constraints
	static float			sFrameBendThreshold;    // max deformation angle (rad) before a frame constraint breaks
	static float			sFrameSpringStiffness;  // N·m/rad — rotational stiffness of frame joints
	static float			sFrameSpringDamping;    // N·m·s/rad — rotational damping of frame joints
	static float			sFrameYieldAngle;       // rad — bend angle above which a joint is plastically yielding
	static float			sFrameYieldTimeLimit;   // s — sustained yield duration before the joint fractures
	static float			sStructuralMassCap;     // kg — effective-mass ceiling for non-projectile impact damage (damage = relV * min(m_reduced, cap))
};
