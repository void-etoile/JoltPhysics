// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Samples.h>

#include <Tests/General/DestructibleTest.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>
#include <Jolt/Physics/Body/BodyLock.h>
#include <Jolt/Core/Profiler.h>
#include <chrono>
#include <algorithm>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Constraints/SixDOFConstraint.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Application/DebugUI.h>
#include <Input/Keyboard.h>
#include <Layers.h>

JPH_IMPLEMENT_RTTI_VIRTUAL(DestructibleTest)
{
	JPH_ADD_BASE_CLASS(DestructibleTest, Test)
}

// ---------------------------------------------------------------------------
// TrackConstraint / UntrackConstraint
// ---------------------------------------------------------------------------

void DestructibleTest::TrackConstraint(bool inIsFrame, Body *inA, Body *inB)
{
	if (inIsFrame)
	{
		SixDOFConstraintSettings s;
		s.mSpace = EConstraintSpace::LocalToBodyCOM;

		RVec3 posA = inA->GetCenterOfMassPosition();
		RVec3 posB = inB->GetCenterOfMassPosition();
		Quat  rotA = inA->GetRotation();
		Quat  rotB = inB->GetRotation();

		Vec3 halfOffset = Vec3(posB - posA) * 0.5f;
		s.mPosition1 = rotA.Conjugated() * halfOffset;
		s.mPosition2 = rotB.Conjugated() * -halfOffset;

		s.mAxisX1 = s.mAxisX2 = Vec3::sAxisX();
		s.mAxisY1 = s.mAxisY2 = Vec3::sAxisY();

		using EA = SixDOFConstraintSettings::EAxis;
		s.MakeFixedAxis(EA::TranslationX);
		s.MakeFixedAxis(EA::TranslationY);
		s.MakeFixedAxis(EA::TranslationZ);
		s.MakeFreeAxis(EA::RotationX);
		s.MakeFreeAxis(EA::RotationY);
		s.MakeFreeAxis(EA::RotationZ);

		for (int ax = (int)EA::RotationX; ax <= (int)EA::RotationZ; ++ax)
		{
			s.mMotorSettings[ax].mSpringSettings.mMode      = ESpringMode::StiffnessAndDamping;
			s.mMotorSettings[ax].mSpringSettings.mStiffness  = sFrameSpringStiffness;
			s.mMotorSettings[ax].mSpringSettings.mDamping    = sFrameSpringDamping;
			s.mMotorSettings[ax].mMaxForceLimit              =  sFrameBreakMoment;
			s.mMotorSettings[ax].mMinForceLimit              = -sFrameBreakMoment;
		}

		Ref<SixDOFConstraint> c = StaticCast<SixDOFConstraint>(s.Create(*inA, *inB));
		mPhysicsSystem->AddConstraint(c);

		using EC = SixDOFConstraint::EAxis;
		c->SetMotorState(EC::RotationX, EMotorState::Position);
		c->SetMotorState(EC::RotationY, EMotorState::Position);
		c->SetMotorState(EC::RotationZ, EMotorState::Position);
		c->SetTargetOrientationCS(rotA.Conjugated() * rotB);

		mFrameIdx[c.GetPtr()] = (int)mFrameConstraints.size();
		mFrameConstraints.push_back(c);
		BodyID id1 = inA->GetID(), id2 = inB->GetID();
		mFrameConnCount[id1]++;
		mFrameConnCount[id2]++;
		mFrameAdj[id1].push_back(c);
		mFrameAdj[id2].push_back(c);
		mConstraintRestRot[c.GetPtr()] = rotA.Conjugated() * rotB;
	}
	else
	{
		FixedConstraintSettings s;
		s.mAutoDetectPoint = true;
		Ref<FixedConstraint> c = StaticCast<FixedConstraint>(s.Create(*inA, *inB));
		mPhysicsSystem->AddConstraint(c);

		mPanelIdx[c.GetPtr()] = (int)mPanelConstraints.size();
		mPanelConstraints.push_back(c);
		BodyID id1 = inA->GetID(), id2 = inB->GetID();
		mPanelConnCount[id1]++;
		mPanelConnCount[id2]++;
		mPanelAdj[id1].push_back(c);
		mPanelAdj[id2].push_back(c);
	}
}

void DestructibleTest::UntrackConstraint(bool inIsFrame, int inPos)
{
	if (inIsFrame)
	{
		SixDOFConstraint *c = mFrameConstraints[inPos].GetPtr();
		mConstraintRestRot.erase(c);
		mJointYieldTime.erase(c);
		BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();

		mRecentlyBrokenFrameBodies.insert(id1);
		mRecentlyBrokenFrameBodies.insert(id2);
		mConnDirty.insert(id1);
		mConnDirty.insert(id2);

		if (--mFrameConnCount[id1] == 0) mFrameConnCount.erase(id1);
		if (--mFrameConnCount[id2] == 0) mFrameConnCount.erase(id2);

		auto removeAdj = [&](BodyID id)
		{
			auto it = mFrameAdj.find(id);
			if (it == mFrameAdj.end()) return;
			Array<Ref<SixDOFConstraint>> &list = it->second;
			for (int j = 0; j < (int)list.size(); ++j)
			{
				if (list[j].GetPtr() == c)
				{
					if (j + 1 < (int)list.size())
						list[j] = std::move(list.back());
					list.pop_back();
					break;
				}
			}
			if (list.empty()) mFrameAdj.erase(it);
		};
		removeAdj(id1);
		removeAdj(id2);

		mFrameIdx.erase(c);
		mPhysicsSystem->RemoveConstraint(c);

		int last = (int)mFrameConstraints.size() - 1;
		if (inPos != last)
		{
			mFrameConstraints[inPos] = std::move(mFrameConstraints[last]);
			mFrameIdx[mFrameConstraints[inPos].GetPtr()] = inPos;
		}
		mFrameConstraints.pop_back();
	}
	else
	{
		FixedConstraint *c = mPanelConstraints[inPos].GetPtr();
		BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();
		mConnDirty.insert(id1);
		mConnDirty.insert(id2);

		if (--mPanelConnCount[id1] == 0) mPanelConnCount.erase(id1);
		if (--mPanelConnCount[id2] == 0) mPanelConnCount.erase(id2);

		auto removeAdj = [&](BodyID id)
		{
			auto it = mPanelAdj.find(id);
			if (it == mPanelAdj.end()) return;
			Array<Ref<FixedConstraint>> &list = it->second;
			for (int j = 0; j < (int)list.size(); ++j)
			{
				if (list[j].GetPtr() == c)
				{
					if (j + 1 < (int)list.size())
						list[j] = std::move(list.back());
					list.pop_back();
					break;
				}
			}
			if (list.empty()) mPanelAdj.erase(it);
		};
		removeAdj(id1);
		removeAdj(id2);

		mPanelIdx.erase(c);
		mPhysicsSystem->RemoveConstraint(c);

		int last = (int)mPanelConstraints.size() - 1;
		if (inPos != last)
		{
			mPanelConstraints[inPos] = std::move(mPanelConstraints[last]);
			mPanelIdx[mPanelConstraints[inPos].GetPtr()] = inPos;
		}
		mPanelConstraints.pop_back();
	}
}

// ---------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------

float DestructibleTest::sPanelBreakForce     =    50.0f;
float DestructibleTest::sFloorBreakForce     =   300.0f;  // N·s — floor slabs need much more damage than thin panels
float DestructibleTest::sFrameBreakForce     =   500.0f;
float DestructibleTest::sFrameBreakMoment    =  5000.0f;  // N·m — motor yield torque; baked at constraint creation
float DestructibleTest::sFrameBreakAxial     =  2000.0f;  // N  — lateral shear force to break a joint
float DestructibleTest::sFrameBendThreshold  =     0.20f; // rad — deformation angle before a joint breaks
// Stiff enough to feel like concrete/steel (joint deflects a fraction of a degree under working
// load before its motor saturates at sFrameBreakMoment, then flows plastically and the yield/bend
// checks take over) but kept at the original known-stable value: pushing stiffness much higher
// makes the spring's effective frequency exceed what the solver can integrate at 60 Hz for these
// body inertias, which injects energy and sends bodies flying. Overdamped so there is no wobble.
float DestructibleTest::sFrameSpringStiffness = 500000.0f; // N·m/rad
float DestructibleTest::sFrameSpringDamping  = 300000.0f;  // N·m·s/rad
float DestructibleTest::sFrameSwayBreakRate  =     1.0f;  // rad/s — snap joints that are already whipping
float DestructibleTest::sFrameYieldAngle     =     0.04f; // rad — past the elastic limit (≈ break moment / stiffness): joint is yielding
float DestructibleTest::sFrameYieldTimeLimit =     0.5f;  // s — a joint held in sustained yield this long fractures
// Impact damage = relV * min(m_reduced, sStructuralMassCap). Capping the EFFECTIVE MASS (not the
// final impulse) keeps damage proportional to impact velocity — a hard ground strike still breaks
// frame elements (so falling sections shatter on landing) while gentle settling does not — but a
// very heavy body can't produce the unbounded damage that previously pulverized whole sections in
// one frame. With cap 80 kg: a frame element (threshold 500 N·s) breaks at relV ≳ 6 m/s, survives
// a slow settle; panels/floors (lower thresholds) break at correspondingly lower speeds.
float DestructibleTest::sStructuralMassCap   =    80.0f;  // kg

// ---------------------------------------------------------------------------
// Initialize
// ---------------------------------------------------------------------------

void DestructibleTest::Initialize()
{
	mPanelConstraints.clear();
	mFrameConstraints.clear();
	mFractureData.clear();
	mShardBodies.clear();
	mProjectiles.clear();
	mFire = false;
	mWasFire = false;
	mProjectileSet.clear();
	mFrameConnCount.clear();
	mPanelConnCount.clear();
	mFrameAdj.clear();
	mPanelAdj.clear();
	mFrameIdx.clear();
	mPanelIdx.clear();
	mChunkDamage.clear();
	mConstraintRestRot.clear();
	mJointYieldTime.clear();
	mRecentlyBrokenFrameBodies.clear();
	mConnDirty.clear();
	mNextBuildingGroupID = 1;
	{
		std::lock_guard<std::mutex> lock(mDamageMutex);
		mPendingDamage.clear();
		mHitProjectiles.clear();
	}

	CreateFloor(800.0f);
	BuildMainWall();

	// City block
	BuildHouse      (RVec3(16, 0, -11));
	BuildApartment  (RVec3(26, 0, -11), 3, 3.0f, 2.0f);
	BuildApartment  (RVec3(41, 0, -11), 2, 3.0f, 2.0f);
	BuildHouse      (RVec3(52, 0, -11));
	BuildApartment  (RVec3(16, 0, -25), 2, 3.0f, 2.0f);
	BuildHouse      (RVec3(26, 0, -25));
	BuildHouse      (RVec3(41, 0, -25));
	BuildApartment  (RVec3(52, 0, -25), 4, 3.0f, 2.0f);
	BuildApartment  (RVec3(16, 0, +11), 3, 3.0f, 2.0f);
	BuildHouse      (RVec3(26, 0, +11));
	BuildHouse      (RVec3(41, 0, +11));
	BuildApartment  (RVec3(52, 0, +11), 2, 3.0f, 2.0f);
	BuildHouse      (RVec3(16, 0, +25));
	BuildApartment  (RVec3(26, 0, +25), 2, 3.0f, 2.0f);
	BuildApartment  (RVec3(41, 0, +25), 4, 3.0f, 2.0f);
	BuildHouse      (RVec3(52, 0, +25));

	// Second residential block — wider/taller variants and a suburban house row
	BuildApartment  (RVec3(16, 0, -45), 5, 4.0f, 3.0f);
	BuildApartment  (RVec3(30, 0, -45), 6, 3.0f, 2.5f);
	BuildApartment  (RVec3(44, 0, -45), 3, 4.5f, 2.0f);
	BuildHouse      (RVec3(56, 0, -45));
	BuildHouse      (RVec3(16, 0, +45));
	BuildHouse      (RVec3(26, 0, +45));
	BuildApartment  (RVec3(38, 0, +45), 4, 3.5f, 3.0f);
	BuildApartment  (RVec3(52, 0, +45), 7, 3.0f, 2.0f);

	// Landmark structures
	BuildTower      (RVec3(72, 0, 0), 20, 4.0f, 8);
	BuildHighrise   (RVec3(95, 0, 0), 30, 3, 5.0f, 5.0f);
	BuildTower      (RVec3(118, 0, 0), 12, 3.0f, 6);      // smaller hexagonal tower
	BuildHighrise   (RVec3(140, 0, 0), 18, 2, 6.0f, 6.0f); // squat wide highrise
	BuildEiffelTower(RVec3(230, 0, 0));

	// Castle compound off to one side
	BuildCastle     (RVec3(110, 0, -120), 22.0f);

	// Seed per-chunk damage tracking for every registered fracture body
	for (auto &kv : mFractureData)
		mChunkDamage[kv.first] = 0.0f;

	mInitialPanelCount = (int)mPanelConstraints.size();
	mInitialFrameCount = (int)mFrameConstraints.size();
	mLastBreakCheckUs  = 0.0f;

	// Register ourselves as the contact listener so OnContactAdded fires for all impacts
	mPhysicsSystem->SetContactListener(this);
}

// ---------------------------------------------------------------------------
// ContactListener: accumulate impact impulses on the physics thread
// ---------------------------------------------------------------------------

void DestructibleTest::OnContactPersisted(const Body &inBody1, const Body &inBody2,
	const ContactManifold &inManifold, ContactSettings & /*ioSettings*/)
{
	// Only care about falling structural elements pressing against remaining structure.
	// Uses a higher threshold than OnContactAdded to ignore resting contact noise.
	if (inBody1.GetObjectLayer() == Layers::DEBRIS || inBody2.GetObjectLayer() == Layers::DEBRIS)
		return;

	Vec3  v1   = inBody1.GetLinearVelocity();
	Vec3  v2   = inBody2.GetLinearVelocity();
	float relV = (v1 - v2).Dot(inManifold.mWorldSpaceNormal);

	static constexpr float cMinPersistVelocity = 2.0f; // m/s — filters out resting/settling contact
	if (relV < cMinPersistVelocity)
		return;

	float inv_m1 = inBody1.IsDynamic() ? inBody1.GetMotionProperties()->GetInverseMass() : 0.0f;
	float inv_m2 = inBody2.IsDynamic() ? inBody2.GetMotionProperties()->GetInverseMass() : 0.0f;
	float inv_sum = inv_m1 + inv_m2;
	if (inv_sum <= 0.0f)
		return;

	// Damage scales with impact velocity; effective mass is capped so a heavy section landing
	// breaks elements on a hard strike without pulverizing everything in one frame.
	float impulse = relV * JPH::min(1.0f / inv_sum, sStructuralMassCap);

	std::lock_guard<std::mutex> lock(mDamageMutex);
	mPendingDamage.push_back({ inBody1.GetID(), impulse });
	mPendingDamage.push_back({ inBody2.GetID(), impulse });
}

void DestructibleTest::OnContactAdded(const Body &inBody1, const Body &inBody2,
	const ContactManifold &inManifold, ContactSettings & /*ioSettings*/)
{
	// Debris shards must not apply structural damage — a falling floor slab would otherwise
	// cascade-break every floor below it in a single chain.
	if (inBody1.GetObjectLayer() == Layers::DEBRIS || inBody2.GetObjectLayer() == Layers::DEBRIS)
		return;

	// Relative approach velocity along the contact normal.
	// OnContactAdded fires when bodies first touch; resting contact (relV ≈ 0) is ignored.
	Vec3 v1 = inBody1.GetLinearVelocity();
	Vec3 v2 = inBody2.GetLinearVelocity();
	float relV = (v1 - v2).Dot(inManifold.mWorldSpaceNormal); // positive = approaching (normal points body1→body2)

	static constexpr float cMinImpactVelocity = 0.5f; // m/s — ignore settling noise
	if (relV < cMinImpactVelocity)
		return;

	// Approximate impulse J = m_reduced × relV  (N·s)
	// IsDynamic() is false for static and kinematic bodies; those get infinite effective mass.
	float inv_m1 = inBody1.IsDynamic() ? inBody1.GetMotionProperties()->GetInverseMass() : 0.0f;
	float inv_m2 = inBody2.IsDynamic() ? inBody2.GetMotionProperties()->GetInverseMass() : 0.0f;
	float inv_sum = inv_m1 + inv_m2;
	if (inv_sum <= 0.0f)
		return;

	BodyID id1 = inBody1.GetID(), id2 = inBody2.GetID();
	bool isProjectile = mProjectileSet.find(id1) != mProjectileSet.end()
		|| mProjectileSet.find(id2) != mProjectileSet.end();

	float m_reduced = 1.0f / inv_sum;
	float impulse;
	if (isProjectile)
		// A direct projectile hit must reliably cut frame elements — use full reduced mass and
		// guarantee at least enough damage to exceed the frame break threshold.
		impulse = JPH::max(relV * m_reduced, sFrameBreakForce * 1.2f);
	else
		// Structural collision (falling section landing on the ground or lower structure):
		// damage scales with impact velocity, effective mass capped so hard strikes break
		// elements but a slow settle does not, and nothing pulverizes in a single frame.
		impulse = relV * JPH::min(m_reduced, sStructuralMassCap);

	// Push to staging buffer — drained on the main thread in PrePhysicsUpdate.
	std::lock_guard<std::mutex> lock(mDamageMutex);
	mPendingDamage.push_back({ inBody1.GetID(), impulse });
	mPendingDamage.push_back({ inBody2.GetID(), impulse });

	// If either body is a projectile, schedule immediate removal.
	// mProjectileSet is written only on the main thread between steps, so reading here is safe.
	if (mProjectileSet.find(id1) != mProjectileSet.end()) mHitProjectiles.push_back(id1);
	if (mProjectileSet.find(id2) != mProjectileSet.end()) mHitProjectiles.push_back(id2);
}

// ---------------------------------------------------------------------------
// Input
// ---------------------------------------------------------------------------

void DestructibleTest::ProcessInput(const ProcessInputParams &inParams)
{
	mFire = inParams.mKeyboard->IsKeyPressedAndTriggered(EKey::Return, mWasFire);
}

void DestructibleTest::SaveInputState(StateRecorder &inStream) const { inStream.Write(mFire); }
void DestructibleTest::RestoreInputState(StateRecorder &inStream)    { inStream.Read(mFire); }

// ---------------------------------------------------------------------------
// FireProjectile
// ---------------------------------------------------------------------------

void DestructibleTest::FireProjectile(RVec3Arg inPos, Vec3Arg inDirection)
{
	static constexpr float cProjectileSpeed = 20.0f;

	BodyCreationSettings proj(new SphereShape(0.25f),
		inPos, Quat::sIdentity(),
		EMotionType::Dynamic, Layers::MOVING);
	proj.mOverrideMassProperties       = EOverrideMassProperties::CalculateInertia;
	proj.mMassPropertiesOverride.mMass = 50.0f;
	proj.mLinearVelocity               = inDirection.Normalized() * cProjectileSpeed;

	BodyID id = mBodyInterface->CreateAndAddBody(proj, EActivation::Activate);
	mProjectiles.push_back({ id, 3.0f });
	mProjectileSet.insert(id);
}

// ---------------------------------------------------------------------------
// SpawnFracture
// ---------------------------------------------------------------------------

void DestructibleTest::SpawnFracture(BodyID inPanelID)
{
	auto it = mFractureData.find(inPanelID);
	if (it == mFractureData.end())
		return;

	JPH_PROFILE("Destruct:SpawnFracture");
	FractureInfo &info  = it->second;
	RVec3 panel_pos     = mBodyInterface->GetCenterOfMassPosition(inPanelID);
	Quat  panel_rot     = mBodyInterface->GetRotation(inPanelID);
	Vec3  panel_vel     = mBodyInterface->GetLinearVelocity(inPanelID);
	Vec3  panel_ang     = mBodyInterface->GetAngularVelocity(inPanelID);

	static constexpr int cMaxShards = 300;
	const int slots = cMaxShards - (int)mShardBodies.size();

	Array<BodyID> shard_ids;
	shard_ids.reserve(info.mShapes.size());

	for (int i = 0; i < (int)info.mShapes.size() && i < slots; ++i)
	{
		RVec3 frag_pos = panel_pos + RVec3(panel_rot * info.mLocalCenters[i]);

		BodyCreationSettings bcs(info.mShapes[i], frag_pos, panel_rot,
			EMotionType::Dynamic, Layers::DEBRIS);
		bcs.mOverrideMassProperties       = EOverrideMassProperties::CalculateInertia;
		bcs.mMassPropertiesOverride.mMass = 3.5f;
		bcs.mLinearVelocity               = panel_vel;
		bcs.mAngularVelocity              = panel_ang;
		bcs.mLinearDamping                = 0.05f;
		bcs.mAngularDamping               = 0.4f;

		Body *b = mBodyInterface->CreateBody(bcs);
		if (b != nullptr)
			shard_ids.push_back(b->GetID());
	}

	if (!shard_ids.empty())
	{
		BodyInterface::AddState add_state = mBodyInterface->AddBodiesPrepare(shard_ids.data(), (int)shard_ids.size());
		mBodyInterface->AddBodiesFinalize(shard_ids.data(), (int)shard_ids.size(), add_state, EActivation::Activate);
		for (BodyID id : shard_ids)
			mShardBodies.push_back(id);
	}

	// Remove all frame constraints referencing this body via adjacency — O(degree).
	{
		auto adjIt = mFrameAdj.find(inPanelID);
		if (adjIt != mFrameAdj.end())
		{
			Array<Ref<SixDOFConstraint>> toRemove = adjIt->second;
			for (auto &cref : toRemove)
			{
				SixDOFConstraint *c = cref.GetPtr();
				BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();
				mBodyInterface->ActivateBody(id1 == inPanelID ? id2 : id1);
				auto posIt = mFrameIdx.find(c);
				if (posIt != mFrameIdx.end())
					UntrackConstraint(true, posIt->second);
			}
		}
	}

	// Remove all panel constraints referencing this body via adjacency — O(degree).
	{
		auto adjIt = mPanelAdj.find(inPanelID);
		if (adjIt != mPanelAdj.end())
		{
			Array<Ref<FixedConstraint>> toRemove = adjIt->second;
			for (auto &cref : toRemove)
			{
				FixedConstraint *c = cref.GetPtr();
				BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();
				mBodyInterface->ActivateBody(id1 == inPanelID ? id2 : id1);
				auto posIt = mPanelIdx.find(c);
				if (posIt != mPanelIdx.end())
					UntrackConstraint(false, posIt->second);
			}
		}
	}

	mBodyInterface->RemoveBody(inPanelID);
	mBodyInterface->DestroyBody(inPanelID);
	mFractureData.erase(it);
	mChunkDamage.erase(inPanelID);
}

// ---------------------------------------------------------------------------
// ExpandToComponents — BFS from inSeeds through inAdj; writes all reachable bodies into outScope
// ---------------------------------------------------------------------------

void DestructibleTest::ExpandToComponents(const UnorderedSet<BodyID> &inSeeds,
	const UnorderedMap<BodyID, Array<int>> &inAdj,
	UnorderedSet<BodyID> &outScope) const
{
	Array<BodyID> queue;
	for (BodyID seed : inSeeds)
	{
		if (inAdj.find(seed) == inAdj.end()) continue;
		if (!outScope.insert(seed).second) continue;
		queue.push_back(seed);
	}
	for (int qi = 0; qi < (int)queue.size(); ++qi)
	{
		BodyID cur = queue[qi];
		auto it = inAdj.find(cur);
		if (it == inAdj.end()) continue;
		for (int ci : it->second)
		{
			SixDOFConstraint *c = mFrameConstraints[ci];
			BodyID other = (c->GetBody1()->GetID() == cur) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
			if (outScope.insert(other).second)
				queue.push_back(other);
		}
	}
}

// ---------------------------------------------------------------------------
// BuildFrameAdjacency — shared adjacency map used by all structural checks
// ---------------------------------------------------------------------------

UnorderedMap<BodyID, Array<int>> DestructibleTest::BuildFrameAdjacency() const
{
	UnorderedMap<BodyID, Array<int>> adj;
	adj.reserve((uint32)mFrameConstraints.size() * 2);
	for (int i = 0; i < (int)mFrameConstraints.size(); ++i)
	{
		SixDOFConstraint *c = mFrameConstraints[i];
		adj[c->GetBody1()->GetID()].push_back(i);
		adj[c->GetBody2()->GetID()].push_back(i);
	}
	return adj;
}

void DestructibleTest::ComputeGroundedSet(const UnorderedMap<BodyID, Array<int>> &inAdj, UnorderedSet<BodyID> &outGrounded)
{
	BodyInterface &binl = mPhysicsSystem->GetBodyInterfaceNoLock();
	Array<BodyID> queue;
	for (auto &kv : inAdj)
		if (binl.GetMotionType(kv.first) == EMotionType::Static)
			if (outGrounded.insert(kv.first).second)
				queue.push_back(kv.first);
	for (int qi = 0; qi < (int)queue.size(); ++qi)
	{
		auto it = inAdj.find(queue[qi]);
		if (it == inAdj.end()) continue;
		for (int ci : it->second)
		{
			SixDOFConstraint *c = mFrameConstraints[ci];
			BodyID other = (c->GetBody1()->GetID() == queue[qi]) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
			if (outGrounded.insert(other).second)
				queue.push_back(other);
		}
	}
}

// ---------------------------------------------------------------------------
// CheckStructuralIntegrity — BFS from ground; detach unsupported chunks
// ---------------------------------------------------------------------------

void DestructibleTest::CheckStructuralIntegrity(const UnorderedSet<BodyID> &inScope)
{
	JPH_PROFILE("Destruct:StructuralIntegrity");
	if (mFrameConstraints.empty())
		return;

	BodyInterface &binl = mPhysicsSystem->GetBodyInterfaceNoLock();

	auto adj = BuildFrameAdjacency();

	UnorderedSet<BodyID> reachable;
	Array<BodyID> queue;

	for (auto &kv : adj)
		if (binl.GetMotionType(kv.first) == EMotionType::Static)
			if (reachable.insert(kv.first).second)
				queue.push_back(kv.first);

	for (int qi = 0; qi < (int)queue.size(); ++qi)
	{
		BodyID current = queue[qi];
		auto it = adj.find(current);
		if (it == adj.end()) continue;
		for (int ci : it->second)
		{
			SixDOFConstraint *c = mFrameConstraints[ci];
			BodyID other = (c->GetBody1()->GetID() == current) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
			if (reachable.insert(other).second)
				queue.push_back(other);
		}
	}

	UnorderedSet<BodyID> unsupported;
	for (auto &kv : adj)
		if (reachable.find(kv.first) == reachable.end() && inScope.find(kv.first) != inScope.end())
			unsupported.insert(kv.first);

	if (unsupported.empty())
		return;

	// Only sever BOUNDARY frame connections (one endpoint supported, one not).
	for (int i = 0; i < (int)mFrameConstraints.size(); )
	{
		SixDOFConstraint *c = mFrameConstraints[i].GetPtr();
		bool b1 = unsupported.find(c->GetBody1()->GetID()) != unsupported.end();
		bool b2 = unsupported.find(c->GetBody2()->GetID()) != unsupported.end();
		if (b1 != b2)
			UntrackConstraint(true, i);
		else
			++i;
	}

	// Panel constraints are left intact — panels fall with their frame section and detach
	// naturally via the 5 cm gap check as the section moves.

	for (BodyID id : unsupported)
		mBodyInterface->ActivateBody(id);
}

// ---------------------------------------------------------------------------
// CheckGravitationalMoment — Tarjan O(N+E) bridge finding
// For each frame constraint that is a structural bridge (sole path from ground
// to a hanging subtree), compute the gravitational bending moment:
//   M = total_hanging_mass * g * horizontal_COM_offset_from_constraint
// If M > sFrameBreakMoment the constraint snaps.  This is purely geometric —
// immune to solver-impulse spikes from projectile impacts.
// ---------------------------------------------------------------------------

void DestructibleTest::CheckGravitationalMoment(const UnorderedSet<BodyID> &inScope)
{
	JPH_PROFILE("Destruct:GravitationalMoment");
	if (mFrameConstraints.empty())
		return;

	static constexpr float cGravity = 9.81f;

	BodyInterface &binl = mPhysicsSystem->GetBodyInterfaceNoLock();

	auto adj = BuildFrameAdjacency();

	const BodyLockInterfaceNoLock &bli = mPhysicsSystem->GetBodyLockInterfaceNoLock();

	UnorderedMap<BodyID, int>   disc, low;
	UnorderedMap<BodyID, float> stMass;
	UnorderedMap<BodyID, Vec3>  stComW;
	disc.reserve((uint32)adj.size());
	low.reserve((uint32)adj.size());
	stMass.reserve((uint32)adj.size());
	stComW.reserve((uint32)adj.size());

	int        timer = 0;
	Array<int> toBreak;

	struct StackFrame { BodyID node; int parentEdge; int adjIdx; };
	Array<StackFrame> stack;

	for (auto &startKV : adj)
	{
		BodyID seed = startKV.first;
		if (disc.find(seed) != disc.end()) continue;
		if (binl.GetMotionType(seed) != EMotionType::Static) continue;
		if (inScope.find(seed) == inScope.end()) continue;

		disc[seed] = low[seed] = timer++;
		stMass[seed] = 0.0f;
		stComW[seed] = Vec3::sZero();
		stack.push_back({ seed, -1, 0 });

		while (!stack.empty())
		{
			BodyID cur    = stack.back().node;
			bool   pushed = false;

			auto adjIt = adj.find(cur);
			if (adjIt != adj.end())
			{
				auto  &adjList = adjIt->second;
				int   &adjIdx  = stack.back().adjIdx;

				while (adjIdx < (int)adjList.size())
				{
					int ci = adjList[adjIdx++];
					if (ci == stack.back().parentEdge) continue;

					SixDOFConstraint *c    = mFrameConstraints[ci];
					BodyID           next = (c->GetBody1()->GetID() == cur) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();

					auto dIt = disc.find(next);
					if (dIt != disc.end())
					{
						low[cur] = JPH::min(low[cur], dIt->second);
						continue;
					}

					disc[next] = low[next] = timer++;
					{
						float mass = 0.0f;
						BodyLockRead lock(bli, next);
						if (lock.Succeeded() && lock.GetBody().IsDynamic())
							mass = 1.0f / lock.GetBody().GetMotionProperties()->GetInverseMass();
						RVec3 pos = binl.GetCenterOfMassPosition(next);
						stMass[next] = mass;
						stComW[next] = Vec3(pos) * mass;
					}
					stack.push_back({ next, ci, 0 });
					pushed = true;
					break;
				}
			}

			if (!pushed)
			{
				int parentEdge = stack.back().parentEdge;
				stack.pop_back();

				if (!stack.empty())
				{
					BodyID par  = stack.back().node;
					low[par]    = JPH::min(low[par], low[cur]);
					stMass[par] += stMass[cur];
					stComW[par] += stComW[cur];

					// Skip foundation connections (par is a static stub): the building's COM is
					// never directly above a single corner stub, so the moment would always be
					// enormous and every base joint would break immediately.
					const bool parIsStatic = binl.GetMotionType(par) == EMotionType::Static;
					if (!parIsStatic && low[cur] > disc[par] && stMass[cur] > 0.0f)
					{
						SixDOFConstraint *bridgeC = mFrameConstraints[parentEdge];
						RVec3 posA     = binl.GetCenterOfMassPosition(bridgeC->GetBody1()->GetID());
						RVec3 posB     = binl.GetCenterOfMassPosition(bridgeC->GetBody2()->GetID());
						RVec3 attachPt = (posA + posB) * 0.5f;

						Vec3  comWorld  = stComW[cur] / stMass[cur];
						Vec3  comOffset = comWorld - Vec3(attachPt);
						float horizOff  = Vec3(comOffset.GetX(), 0.0f, comOffset.GetZ()).Length();
						float axial     = stMass[cur] * cGravity;
						float moment    = axial * horizOff;

						if (moment > sFrameBreakMoment)
							toBreak.push_back(parentEdge);
					}
				}
			}
		}
	}

	std::sort(toBreak.begin(), toBreak.end(), std::greater<int>());
	for (int ci : toBreak)
	{
		if (ci >= (int)mFrameConstraints.size()) continue;
		SixDOFConstraint *c = mFrameConstraints[ci];
		mBodyInterface->ActivateBody(c->GetBody1()->GetID());
		mBodyInterface->ActivateBody(c->GetBody2()->GetID());
		UntrackConstraint(true, ci);
	}
}

// ---------------------------------------------------------------------------
// CheckSupportStability
// Fractures frame bodies that are geometrically unsupported: a body is
// unsupported when its XZ position lies outside the bounding box of the
// static stubs (ground-anchored segments) in its connected component.
// This catches short structures (house, 2-storey apt) whose roofs/beams
// overhang the remaining columns after one wall is destroyed — the bridge
// check misses them because two support paths still exist.
// ---------------------------------------------------------------------------

void DestructibleTest::CheckSupportStability(const UnorderedSet<BodyID> &inScope)
{
	JPH_PROFILE("Destruct:SupportStability");
	if (mFractureData.empty() || mFrameConstraints.empty())
		return;

	BodyInterface &binl = mPhysicsSystem->GetBodyInterfaceNoLock();

	auto adj = BuildFrameAdjacency();

	// BFS: find connected components; compute XZ bbox of static stubs per component.
	struct CompInfo { float minX, maxX, minZ, maxZ; bool hasStub; };
	UnorderedMap<BodyID, int> bodyToComp;
	Array<CompInfo>           comps;
	bodyToComp.reserve((uint32)adj.size());

	for (auto &kv : adj)
	{
		BodyID seed = kv.first;
		if (bodyToComp.find(seed) != bodyToComp.end()) continue;

		int compIdx = (int)comps.size();
		comps.push_back({ FLT_MAX, -FLT_MAX, FLT_MAX, -FLT_MAX, false });
		CompInfo &ci = comps.back();

		Array<BodyID> queue;
		queue.push_back(seed);
		bodyToComp[seed] = compIdx;

		for (int qi = 0; qi < (int)queue.size(); ++qi)
		{
			BodyID cur = queue[qi];

			if (binl.GetMotionType(cur) == EMotionType::Static)
			{
				RVec3 pos = binl.GetCenterOfMassPosition(cur);
				float px = (float)pos.GetX(), pz = (float)pos.GetZ();
				if (px < ci.minX) ci.minX = px;
				if (px > ci.maxX) ci.maxX = px;
				if (pz < ci.minZ) ci.minZ = pz;
				if (pz > ci.maxZ) ci.maxZ = pz;
				ci.hasStub = true;
			}

			auto it = adj.find(cur);
			if (it == adj.end()) continue;
			for (int k : it->second)
			{
				SixDOFConstraint *c = mFrameConstraints[k];
				BodyID other = (c->GetBody1()->GetID() == cur) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
				if (bodyToComp.find(other) == bodyToComp.end())
				{
					bodyToComp[other] = compIdx;
					queue.push_back(other);
				}
			}
		}
	}

	// Tolerance: allow a body to be slightly outside the stub bbox before fracturing.
	// 0.5 m covers the half-width of a typical column stub so corner columns don't
	// false-positive when the bbox is a single point.
	static constexpr float cTol = 0.5f;
	static constexpr int   cMaxFracturesPerFrame = 4;

	Array<BodyID> toFracture;
	for (auto &kv : mFractureData)
	{
		if (!kv.second.mIsFrame) continue;
		BodyID id = kv.first;

		if (inScope.find(id) == inScope.end()) continue;

		auto compIt = bodyToComp.find(id);
		if (compIt == bodyToComp.end()) continue;

		const CompInfo &ci = comps[compIt->second];
		if (!ci.hasStub) continue;

		RVec3 pos = binl.GetCenterOfMassPosition(id);
		float bx = (float)pos.GetX(), bz = (float)pos.GetZ();
		if (bx < ci.minX - cTol || bx > ci.maxX + cTol ||
		    bz < ci.minZ - cTol || bz > ci.maxZ + cTol)
			toFracture.push_back(id);
	}

	int budget = cMaxFracturesPerFrame;
	for (BodyID id : toFracture)
	{
		if (budget-- <= 0) break;
		if (mFractureData.find(id) == mFractureData.end()) continue;
		mBodyInterface->ActivateBody(id);
		SpawnFracture(id);
	}
}

// ---------------------------------------------------------------------------
// PrePhysicsUpdate
// ---------------------------------------------------------------------------

void DestructibleTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	JPH_PROFILE("Destruct:PrePhysicsUpdate");
	BodyInterface &binl = mPhysicsSystem->GetBodyInterfaceNoLock();

	// ---- Fire projectile ----
	if (mFire)
	{
		RVec3 start = inParams.mCameraState.mPos + RVec3(inParams.mCameraState.mForward);
		FireProjectile(start, inParams.mCameraState.mForward.Normalized());
		mFire = false;
	}

	// ---- Expire projectiles (lifetime OR on-impact removal) ----
	{
		std::lock_guard<std::mutex> lock(mDamageMutex);
		for (BodyID hitID : mHitProjectiles)
			for (auto &rec : mProjectiles)
				if (rec.mID == hitID) { rec.mLifeRemaining = 0.0f; break; }
		mHitProjectiles.clear();
	}
	for (int i = (int)mProjectiles.size() - 1; i >= 0; --i)
	{
		mProjectiles[i].mLifeRemaining -= inParams.mDeltaTime;
		if (mProjectiles[i].mLifeRemaining <= 0.0f)
		{
			mProjectileSet.erase(mProjectiles[i].mID);
			mBodyInterface->RemoveBody(mProjectiles[i].mID);
			mBodyInterface->DestroyBody(mProjectiles[i].mID);
			mProjectiles[i] = std::move(mProjectiles.back());
			mProjectiles.pop_back();
		}
	}

	// ---- Sleep shards ----
	for (int i = (int)mShardBodies.size() - 1; i >= 0; --i)
	{
		if (!binl.IsActive(mShardBodies[i]))
		{
			mBodyInterface->RemoveBody(mShardBodies[i]);
			mBodyInterface->DestroyBody(mShardBodies[i]);
			mShardBodies[i] = mShardBodies.back();
			mShardBodies.pop_back();
		}
	}

	// A frame joint must have broken for any structural section to come loose and fall/bend;
	// until then the whole-scene structural passes (and the below-ground scan over every chunk)
	// have nothing to act on. Computed here so the cleanup below can use it too.
	const bool anyDamage = (int)mFrameConstraints.size() < mInitialFrameCount;

	// ---- Below-ground cleanup: remove anything that has fallen through the floor ----
	{
		JPH_PROFILE("Destruct:BelowGround");
		static constexpr float cFallThreshold = -10.0f;

		for (int i = (int)mProjectiles.size() - 1; i >= 0; --i)
		{
			if ((float)binl.GetCenterOfMassPosition(mProjectiles[i].mID).GetY() < cFallThreshold)
			{
				mProjectileSet.erase(mProjectiles[i].mID);
				mBodyInterface->RemoveBody(mProjectiles[i].mID);
				mBodyInterface->DestroyBody(mProjectiles[i].mID);
				mProjectiles[i] = std::move(mProjectiles.back());
				mProjectiles.pop_back();
			}
		}

		for (int i = (int)mShardBodies.size() - 1; i >= 0; --i)
		{
			if ((float)binl.GetCenterOfMassPosition(mShardBodies[i]).GetY() < cFallThreshold)
			{
				mBodyInterface->RemoveBody(mShardBodies[i]);
				mBodyInterface->DestroyBody(mShardBodies[i]);
				mShardBodies[i] = mShardBodies.back();
				mShardBodies.pop_back();
			}
		}

		// Structural/panel bodies — only freed sections (which exist only after a frame joint
		// breaks) can sink through the floor; an attached chunk is held above ground. So skip the
		// whole-scene position scan entirely until there is damage.
		Array<BodyID> sunken;
		if (anyDamage)
			for (auto &kv : mFractureData)
				if ((float)binl.GetCenterOfMassPosition(kv.first).GetY() < cFallThreshold)
					sunken.push_back(kv.first);

		for (BodyID id : sunken)
		{
			if (mFractureData.find(id) == mFractureData.end()) continue;

			auto frameIt = mFrameAdj.find(id);
			if (frameIt != mFrameAdj.end())
			{
				Array<Ref<SixDOFConstraint>> cs = frameIt->second;
				for (auto &cr : cs)
				{
					auto pit = mFrameIdx.find(cr.GetPtr());
					if (pit != mFrameIdx.end())
						UntrackConstraint(true, pit->second);
				}
			}
			auto panelIt = mPanelAdj.find(id);
			if (panelIt != mPanelAdj.end())
			{
				Array<Ref<FixedConstraint>> cs = panelIt->second;
				for (auto &cr : cs)
				{
					auto pit = mPanelIdx.find(cr.GetPtr());
					if (pit != mPanelIdx.end())
						UntrackConstraint(false, pit->second);
				}
			}
			mBodyInterface->RemoveBody(id);
			mBodyInterface->DestroyBody(id);
			mFractureData.erase(id);
			mChunkDamage.erase(id);
		}
	}

	if (mPhysicsSystem->GetNumActiveBodies(EBodyType::RigidBody) == 0)
		return;

	auto t0 = std::chrono::high_resolution_clock::now();

	// ---- Apply pending contact damage ----
	// Damage only ever increases here, so a chunk can only cross its threshold on a frame it
	// received damage. Collect those bodies and check only them below (instead of the whole scene).
	UnorderedSet<BodyID> damagedThisFrame;
	{
		JPH_PROFILE("Destruct:ApplyDamage");
		std::lock_guard<std::mutex> lock(mDamageMutex);
		for (auto &[id, impulse] : mPendingDamage)
		{
			auto it = mChunkDamage.find(id);
			if (it != mChunkDamage.end())
			{
				it->second += impulse;
				damagedThisFrame.insert(id);
			}
		}
		mPendingDamage.clear();
	}

	// ---- Destroy chunks that exceeded their damage threshold ----
	{
		JPH_PROFILE("Destruct:DestroyChunks");
		Array<BodyID> toDestroy;
		for (BodyID id : damagedThisFrame)
		{
			auto dit = mChunkDamage.find(id);
			if (dit == mChunkDamage.end()) continue;
			auto fit = mFractureData.find(id);
			if (fit == mFractureData.end()) continue;
			float threshold = fit->second.mIsFrame ? sFrameBreakForce :
			                  fit->second.mIsFloor ? sFloorBreakForce : sPanelBreakForce;
			if (dit->second >= threshold)
				toDestroy.push_back(id);
		}
		for (BodyID id : toDestroy)
			SpawnFracture(id); // removes body, constraints, and mChunkDamage[id]
	}

	// ---- Panel gap check: break panels pulled >5 cm from attachment ----
	// Only runs once a frame joint has broken — until then the frame is rigid and panels,
	// pinned to it, cannot be pulled apart, so scanning every panel each frame is wasted work.
	if (anyDamage)
	{
		JPH_PROFILE("Destruct:PanelGap");
		static constexpr float cPanelMaxGap = 0.05f;
		for (int i = 0; i < (int)mPanelConstraints.size(); )
		{
			FixedConstraint *c = mPanelConstraints[i].GetPtr();
			Body *body1 = c->GetBody1();
			Body *body2 = c->GetBody2();
			if (!body1->IsActive() && !body2->IsActive()) { ++i; continue; }

			Vec3  local1 = c->GetConstraintToBody1Matrix().GetTranslation();
			Vec3  local2 = c->GetConstraintToBody2Matrix().GetTranslation();
			RVec3 p1     = body1->GetCenterOfMassPosition() + RVec3(body1->GetRotation() * local1);
			RVec3 p2     = body2->GetCenterOfMassPosition() + RVec3(body2->GetRotation() * local2);

			if (float((p2 - p1).LengthSq()) > cPanelMaxGap * cPanelMaxGap)
			{
				mBodyInterface->ActivateBody(body1->GetID());
				mBodyInterface->ActivateBody(body2->GetID());
				UntrackConstraint(false, i);
			}
			else ++i;
		}
	}

	// ---- Isolation check: free frame elements where BOTH endpoints have exactly 1 connection ----
	// A dangling pair can only arise after a frame joint has broken, so this is gated on anyDamage.
	if (anyDamage)
	{
		JPH_PROFILE("Destruct:Isolation");
		for (int i = 0; i < (int)mFrameConstraints.size(); )
		{
			SixDOFConstraint *c = mFrameConstraints[i].GetPtr();
			BodyID id1 = c->GetBody1()->GetID();
			BodyID id2 = c->GetBody2()->GetID();
			auto it1 = mFrameConnCount.find(id1);
			auto it2 = mFrameConnCount.find(id2);
			bool b1_iso = binl.GetMotionType(id1) == EMotionType::Dynamic
				&& it1 != mFrameConnCount.end() && it1->second == 1;
			bool b2_iso = binl.GetMotionType(id2) == EMotionType::Dynamic
				&& it2 != mFrameConnCount.end() && it2->second == 1;
			if (b1_iso && b2_iso)
			{
				mBodyInterface->ActivateBody(id1);
				mBodyInterface->ActivateBody(id2);
				UntrackConstraint(true, i);
			}
			else ++i;
		}
	}

	// Shared frame-graph snapshot (adjacency + ground-reachable set) reused by the deformation
	// and unground passes — both need exactly this. Built on first use and reused unless a frame
	// joint breaks in between (tracked by the constraint count, which only ever decreases here).
	UnorderedMap<BodyID, Array<int>> sharedFrameAdj;
	UnorderedSet<BodyID>             sharedGrounded;
	int snapshotFrameCount = -1;
	auto ensureFrameSnapshot = [&]()
	{
		if (snapshotFrameCount == (int)mFrameConstraints.size())
			return; // still valid — no frame joint broke since it was built
		sharedFrameAdj = BuildFrameAdjacency();
		sharedGrounded.clear();
		ComputeGroundedSet(sharedFrameAdj, sharedGrounded);
		snapshotFrameCount = (int)mFrameConstraints.size();
	};

	// ---- Deformation failure: break frame joints that have bent past the threshold ----
	// Only check joints where at least one endpoint is still grounded (connected to a static stub).
	// Joints between two freely-falling bodies carry no real structural load — any apparent
	// deformation is solver noise that amplifies toward the top of a falling chain.
	if (anyDamage)
	{
		JPH_PROFILE("Destruct:Deformation");
		ensureFrameSnapshot();
		const UnorderedSet<BodyID> &deformGrounded = sharedGrounded;

		for (int i = (int)mFrameConstraints.size() - 1; i >= 0; --i)
		{
			SixDOFConstraint *c = mFrameConstraints[i].GetPtr();
			Body *b1 = c->GetBody1(), *b2 = c->GetBody2();
			// Cheap lock-free flag read; skips the bulk of sleeping joints in untouched buildings.
			if (!b1->IsActive() && !b2->IsActive()) continue;

			// Skip joints where neither side is grounded — they are in freefall together.
			if (deformGrounded.find(b1->GetID()) == deformGrounded.end() &&
			    deformGrounded.find(b2->GetID()) == deformGrounded.end()) continue;

			auto restIt = mConstraintRestRot.find(c);
			if (restIt == mConstraintRestRot.end()) continue;

			Quat  currentRel = b1->GetRotation().Conjugated() * b2->GetRotation();
			Quat  delta      = restIt->second.Conjugated() * currentRel;
			float bendAngle  = 2.0f * acosf(JPH::Clamp(abs(delta.GetW()), 0.0f, 1.0f));
			float swayRate   = (b2->GetAngularVelocity() - b1->GetAngularVelocity()).Length();

			// Sustained-yield accumulator: a joint bent past the elastic limit is plastically
			// yielding.  Accumulate time while yielding, decay quickly while elastic.  This
			// fractures overloaded cantilevers that settle into a stable bent equilibrium below
			// the hard angle threshold (e.g. a corner that lost its columns but is still held by
			// beams + floor slab on the remaining sides).
			float &yieldTime = mJointYieldTime[c];
			if (bendAngle > sFrameYieldAngle)
				yieldTime += inParams.mDeltaTime;
			else
				yieldTime = JPH::max(0.0f, yieldTime - 2.0f * inParams.mDeltaTime);

			bool angleBreak = bendAngle > sFrameBendThreshold;
			bool swayBreak  = swayRate > sFrameSwayBreakRate && bendAngle > 0.05f;
			bool yieldBreak = yieldTime > sFrameYieldTimeLimit;
			if (angleBreak || swayBreak || yieldBreak)
			{
				mBodyInterface->ActivateBody(b1->GetID());
				mBodyInterface->ActivateBody(b2->GetID());
				UntrackConstraint(true, i);
			}
		}
	}

	// ---- Structural cascade: break bridge joints then propagate disconnection ----
	// Scoped to only the connected components containing recently broken frame joints,
	// so an impact on building A cannot trigger cascades in buildings B, C, etc.
	// The scope is kept alive and reused by CheckSupportStability below.
	UnorderedSet<BodyID> cascadeScope;
	if (!mRecentlyBrokenFrameBodies.empty())
	{
		JPH_PROFILE("Destruct:CascadeScope");
		auto cascAdj = BuildFrameAdjacency();
		ExpandToComponents(mRecentlyBrokenFrameBodies, cascAdj, cascadeScope);
		mRecentlyBrokenFrameBodies.clear(); // cascade's own breaks feed next frame
	}
	if (!cascadeScope.empty())
	{
		CheckGravitationalMoment(cascadeScope);
		CheckStructuralIntegrity(cascadeScope);
	}

	// ---- XZ-overhang check: fracture frame elements that have swung outside their support footprint ----
	if (!cascadeScope.empty())
		CheckSupportStability(cascadeScope);

	// ---- Unground check: re-group freed sections so they collide correctly ----
	// A section that can no longer reach a static anchor must collide with the standing
	// structure below it, but NOT with its own connected neighbours (which touch at their
	// joints).  Clearing to the invalid group makes a body collide with EVERYTHING including
	// section-mates, whose overlapping joints then generate huge spurious contact forces that
	// blow the section apart.  Instead, give each connected ungrounded component a fresh shared
	// GroupID (keeping its filter / subgroup): mates share the id so rule "same group+subgroup =
	// no collision" still holds, while a different id from the parent building lets the section
	// collide with the structure it falls onto.
	if (anyDamage && !mFrameConstraints.empty())
	{
		JPH_PROFILE("Destruct:Unground");
		// Reuse the snapshot built by the deformation pass; it is rebuilt here only if a frame
		// joint broke since (deformation / cascade / support-stability), which changes the count.
		ensureFrameSnapshot();
		const UnorderedMap<BodyID, Array<int>> &uadj     = sharedFrameAdj;
		const UnorderedSet<BodyID>             &grounded = sharedGrounded;

		const BodyLockInterfaceNoLock &bli = mPhysicsSystem->GetBodyLockInterfaceNoLock();

		// A panel/floor body counts as grounded if any frame body it is panel-attached to is grounded.
		auto panelGrounded = [&](BodyID id) -> bool
		{
			auto pit = mPanelAdj.find(id);
			if (pit == mPanelAdj.end()) return false;
			for (auto &cref : pit->second)
			{
				FixedConstraint *pc = cref.GetPtr();
				BodyID o = (pc->GetBody1()->GetID() == id) ? pc->GetBody2()->GetID() : pc->GetBody1()->GetID();
				if (grounded.find(o) != grounded.end()) return true;
			}
			return false;
		};

		auto setGroupID = [&](BodyID id, uint32 newGid)
		{
			BodyLockWrite lock(bli, id);
			if (lock.Succeeded())
			{
				CollisionGroup g = lock.GetBody().GetCollisionGroup();
				g.SetGroupID(newGid);
				lock.GetBody().SetCollisionGroup(g);
			}
		};

		// Flood each ungrounded component (through frame + panel links) with one fresh GroupID.
		UnorderedSet<BodyID> visited;
		for (auto &kv : uadj)
		{
			BodyID seed = kv.first;
			if (binl.GetMotionType(seed) != EMotionType::Dynamic) continue;
			if (grounded.find(seed) != grounded.end()) continue;
			if (visited.find(seed) != visited.end()) continue;

			uint32 newGid = mNextBuildingGroupID++;
			Array<BodyID> q;
			q.push_back(seed);
			visited.insert(seed);

			for (int qi = 0; qi < (int)q.size(); ++qi)
			{
				BodyID cur = q[qi];
				setGroupID(cur, newGid);

				auto fit = uadj.find(cur);
				if (fit != uadj.end())
					for (int ci : fit->second)
					{
						SixDOFConstraint *c = mFrameConstraints[ci];
						BodyID o = (c->GetBody1()->GetID() == cur) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
						if (binl.GetMotionType(o) == EMotionType::Dynamic
							&& grounded.find(o) == grounded.end() && visited.insert(o).second)
							q.push_back(o);
					}

				auto pit = mPanelAdj.find(cur);
				if (pit != mPanelAdj.end())
					for (auto &cref : pit->second)
					{
						FixedConstraint *pc = cref.GetPtr();
						BodyID o = (pc->GetBody1()->GetID() == cur) ? pc->GetBody2()->GetID() : pc->GetBody1()->GetID();
						if (binl.GetMotionType(o) == EMotionType::Dynamic
							&& grounded.find(o) == grounded.end() && !panelGrounded(o) && visited.insert(o).second)
							q.push_back(o);
					}
			}
		}
	}

	// ---- Fracture bodies that have lost all their constraints ----
	// Only bodies whose connection count changed this/last frame (mConnDirty) can be newly
	// zero-connection, so we check just those rather than scanning the whole scene every frame.
	if (!mConnDirty.empty())
	{
		JPH_PROFILE("Destruct:ZeroConnection");
		Array<BodyID> to_fracture;
		for (BodyID id : mConnDirty)
		{
			auto fit = mFractureData.find(id);
			if (fit == mFractureData.end()) continue; // already fractured / gone
			const UnorderedMap<BodyID, int> &cnt = fit->second.mIsFrame ? mFrameConnCount : mPanelConnCount;
			if (cnt.find(id) == cnt.end())
				to_fracture.push_back(id);
		}
		mConnDirty.clear();

		// Clear collision group before fracturing so the body can collide during the brief window
		// before it becomes debris — and so debris shards spawn in a state where they can land on structure.
		{
			const BodyLockInterfaceNoLock &bli2 = mPhysicsSystem->GetBodyLockInterfaceNoLock();
			for (BodyID id : to_fracture)
			{
				BodyLockWrite lock(bli2, id);
				if (lock.Succeeded()) lock.GetBody().SetCollisionGroup(CollisionGroup());
			}
		}

		static constexpr int cMaxFracturesPerFrame = 3;
		int budget = cMaxFracturesPerFrame;
		for (BodyID id : to_fracture)
		{
			if (budget-- <= 0)
			{
				mConnDirty.insert(id); // over budget — re-check next frame
				continue;
			}
			SpawnFracture(id);
		}
	}

	auto t1 = std::chrono::high_resolution_clock::now();
	mLastBreakCheckUs = std::chrono::duration<float, std::micro>(t1 - t0).count();
}

// ---------------------------------------------------------------------------
// Status / UI
// ---------------------------------------------------------------------------

String DestructibleTest::GetStatusString() const
{
	char buf[128];
	snprintf(buf, sizeof(buf),
		"Panels: %d / %d  |  Frame: %d / %d  |  Update: %.1f us",
		(int)mPanelConstraints.size(), mInitialPanelCount,
		(int)mFrameConstraints.size(),  mInitialFrameCount,
		mLastBreakCheckUs);
	return buf;
}

void DestructibleTest::CreateSettingsMenu(DebugUI *inUI, UIElement *inSubMenu)
{
	inUI->CreateSlider(inSubMenu, "Panel Break Force (N\xc2\xb7s)", sPanelBreakForce,   5.0f,   200.0f,  5.0f,
		[](float inValue) { sPanelBreakForce = inValue; });

	inUI->CreateSlider(inSubMenu, "Floor Break Force (N\xc2\xb7s)", sFloorBreakForce,  50.0f, 2000.0f, 50.0f,
		[](float inValue) { sFloorBreakForce = inValue; });

	inUI->CreateSlider(inSubMenu, "Frame Break Force (N\xc2\xb7s)", sFrameBreakForce, 100.0f, 2000.0f, 50.0f,
		[](float inValue) { sFrameBreakForce = inValue; });

	inUI->CreateSlider(inSubMenu, "Motor Yield Torque (N\xc2\xb7m, restart)", sFrameBreakMoment, 500.0f, 50000.0f, 500.0f,
		[](float inValue) { sFrameBreakMoment = inValue; });

	inUI->CreateSlider(inSubMenu, "Joint Break Force (N)", sFrameBreakAxial, 100.0f, 20000.0f, 100.0f,
		[](float inValue) { sFrameBreakAxial = inValue; });

	inUI->CreateSlider(inSubMenu, "Bend Threshold (rad)", sFrameBendThreshold, 0.05f, 1.0f, 0.05f,
		[](float inValue) { sFrameBendThreshold = inValue; });

	inUI->CreateSlider(inSubMenu, "Frame Spring Stiffness (N\xc2\xb7m/rad)", sFrameSpringStiffness, 50000.0f, 3000000.0f, 50000.0f,
		[](float inValue) { sFrameSpringStiffness = inValue; });

	inUI->CreateSlider(inSubMenu, "Sway Break Rate (rad/s)", sFrameSwayBreakRate, 0.1f, 5.0f, 0.1f,
		[](float inValue) { sFrameSwayBreakRate = inValue; });

	inUI->CreateSlider(inSubMenu, "Yield Angle (rad)", sFrameYieldAngle, 0.01f, 0.20f, 0.01f,
		[](float inValue) { sFrameYieldAngle = inValue; });

	inUI->CreateSlider(inSubMenu, "Yield Time Limit (s)", sFrameYieldTimeLimit, 0.1f, 3.0f, 0.1f,
		[](float inValue) { sFrameYieldTimeLimit = inValue; });

	inUI->CreateSlider(inSubMenu, "Structural Mass Cap (kg)", sStructuralMassCap, 20.0f, 400.0f, 10.0f,
		[](float inValue) { sStructuralMassCap = inValue; });

	inUI->CreateTextButton(inSubMenu, "Reset", [this]() { RestartTest(); });
}
