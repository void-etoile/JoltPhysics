// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#pragma once

#include <Tests/Test.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Collision/Shape/Shape.h>
#include <Jolt/Physics/Body/BodyID.h>
#include <Jolt/Core/UnorderedMap.h>

/// Demonstrates frame-and-infill destructible environments.
/// A rigid structural frame (columns and beams) is held by high-break-force FixedConstraints.
/// Lightweight infill panels fill each bay with low-break-force constraints so they shatter
/// independently while the frame can survive. Adjust the sliders to explore the difference.
class DestructibleTest : public Test
{
public:
	JPH_DECLARE_RTTI_VIRTUAL(JPH_NO_EXPORT, DestructibleTest)

	// Description of the test
	virtual const char *	GetDescription() const override
	{
		return "A building wall and a house (4 walls + gable roof) using frame-and-infill destruction. "
			   "Panels break at a low threshold; frame constraints require much greater force. "
			   "Press Enter to fire projectiles from the camera. Space to drag bodies. "
			   "Lower the Frame Break Force to collapse the whole structure.";
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

private:
	struct FractureInfo
	{
		Array<RefConst<Shape>>	mShapes;		// one convex hull per Voronoi cell
		Array<Vec3>				mLocalCenters;	// centroid of each cell in panel local space
		bool					mIsFrame = false; // true = fracture when frame constraints gone; false = panel constraints
	};

	void					FireProjectile(RVec3Arg inPos, Vec3Arg inDirection);
	void					BuildHouse(RVec3Arg inCenter);
	void					CheckStructuralIntegrity();
	void					SpawnFracture(BodyID inPanelID);

	struct ProjectileRecord { BodyID mID; float mLifeRemaining; };

	Array<Ref<FixedConstraint>>			mPanelConstraints;	// infill panels — low break force
	Array<Ref<FixedConstraint>>			mFrameConstraints;	// columns and beams — high break force
	UnorderedMap<BodyID, FractureInfo>	mFractureData;		// registered panels → fracture geometry
	Array<BodyID>						mShardBodies;		// all live shard bodies, for sleeping cleanup
	Array<ProjectileRecord>				mProjectiles;		// live projectiles with remaining lifetime

	// Adjacency index: maintained incrementally so constraint lookup/removal is O(degree).
	UnorderedMap<BodyID, int>							mFrameConnCount;	// #frame constraints touching each body
	UnorderedMap<BodyID, int>							mPanelConnCount;	// #panel constraints touching each body
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>>	mFrameAdj;			// frame constraints per body
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>>	mPanelAdj;			// panel constraints per body
	UnorderedMap<FixedConstraint *, int>				mFrameIdx;			// constraint → index in mFrameConstraints
	UnorderedMap<FixedConstraint *, int>				mPanelIdx;			// constraint → index in mPanelConstraints

	// Create a FixedConstraint between inA and inB, add it to the physics system, and
	// register it in all adjacency data structures.
	void					TrackConstraint(bool inIsFrame, Body *inA, Body *inB);

	// Remove the constraint at position inPos from the appropriate array (swap-and-pop),
	// update all adjacency data structures, and call RemoveConstraint.
	void					UntrackConstraint(bool inIsFrame, int inPos);

	int						mInitialPanelCount = 0;
	int						mInitialFrameCount = 0;
	float					mLastBreakCheckUs = 0.0f;	// microseconds for constraint break loop

	bool					mFire = false;			// set in ProcessInput, consumed in PrePhysicsUpdate
	bool					mWasFire = false;		// edge-detection state for IsKeyPressedAndTriggered

	static float			sPanelBreakForce;
	static float			sFrameBreakForce;
};
