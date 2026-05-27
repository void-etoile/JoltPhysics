// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Samples.h>

#include <Tests/General/DestructibleTest.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>
#include <chrono>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
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
	FixedConstraintSettings s;
	s.mAutoDetectPoint = true;
	Ref<FixedConstraint> c = StaticCast<FixedConstraint>(s.Create(*inA, *inB));
	mPhysicsSystem->AddConstraint(c);

	Array<Ref<FixedConstraint>> &arr                         = inIsFrame ? mFrameConstraints : mPanelConstraints;
	UnorderedMap<BodyID, int> &cnt                           = inIsFrame ? mFrameConnCount   : mPanelConnCount;
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adj   = inIsFrame ? mFrameAdj         : mPanelAdj;
	UnorderedMap<FixedConstraint *, int> &idx                = inIsFrame ? mFrameIdx         : mPanelIdx;

	idx[c.GetPtr()] = (int)arr.size();
	arr.push_back(c);
	BodyID id1 = inA->GetID(), id2 = inB->GetID();
	cnt[id1]++;
	cnt[id2]++;
	adj[id1].push_back(c);
	adj[id2].push_back(c);
}

void DestructibleTest::UntrackConstraint(bool inIsFrame, int inPos)
{
	Array<Ref<FixedConstraint>> &arr                         = inIsFrame ? mFrameConstraints : mPanelConstraints;
	UnorderedMap<BodyID, int> &cnt                           = inIsFrame ? mFrameConnCount   : mPanelConnCount;
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adj   = inIsFrame ? mFrameAdj         : mPanelAdj;
	UnorderedMap<FixedConstraint *, int> &idx                = inIsFrame ? mFrameIdx         : mPanelIdx;

	FixedConstraint *c = arr[inPos].GetPtr();
	BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();

	if (--cnt[id1] == 0) cnt.erase(id1);
	if (--cnt[id2] == 0) cnt.erase(id2);

	auto removeAdj = [&](BodyID id)
	{
		auto it = adj.find(id);
		if (it == adj.end()) return;
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
		if (list.empty()) adj.erase(it);
	};
	removeAdj(id1);
	removeAdj(id2);

	idx.erase(c);
	mPhysicsSystem->RemoveConstraint(c);

	int last = (int)arr.size() - 1;
	if (inPos != last)
	{
		arr[inPos] = std::move(arr[last]);
		idx[arr[inPos].GetPtr()] = inPos;
	}
	arr.pop_back();
}

// ---------------------------------------------------------------------------
// Statics
// ---------------------------------------------------------------------------

float DestructibleTest::sPanelBreakForce =   50.0f;
float DestructibleTest::sFrameBreakForce =  500.0f;

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
	mFrameConnCount.clear();
	mPanelConnCount.clear();
	mFrameAdj.clear();
	mPanelAdj.clear();
	mFrameIdx.clear();
	mPanelIdx.clear();
	mChunkDamage.clear();
	{
		std::lock_guard<std::mutex> lock(mDamageMutex);
		mPendingDamage.clear();
	}

	CreateFloor();
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

	// Landmark structures
	BuildTower      (RVec3(72, 0, 0), 20, 4.0f, 8);
	BuildHighrise   (RVec3(95, 0, 0), 30, 3, 5.0f, 5.0f);

	// Seed per-chunk damage tracking for every registered fracture body
	for (auto &kv : mFractureData)
		mChunkDamage[kv.first] = 0.0f;

	mInitialPanelCount = (int)mPanelConstraints.size();
	mInitialFrameCount = (int)mFrameConstraints.size();
	mLastBreakCheckUs  = 0.0f;
	mBreakLogCount     = 0;

	if (mLogFile) fclose(mLogFile);
	mLogFile = fopen("/tmp/jolt_log.txt", "w");

	// Register ourselves as the contact listener so OnContactAdded fires for all impacts
	mPhysicsSystem->SetContactListener(this);
}

// ---------------------------------------------------------------------------
// ContactListener: accumulate impact impulses on the physics thread
// ---------------------------------------------------------------------------

void DestructibleTest::OnContactAdded(const Body &inBody1, const Body &inBody2,
	const ContactManifold &inManifold, ContactSettings & /*ioSettings*/)
{
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

	float impulse = relV / inv_sum;

	// Push to staging buffer — drained on the main thread in PrePhysicsUpdate.
	std::lock_guard<std::mutex> lock(mDamageMutex);
	mPendingDamage.push_back({ inBody1.GetID(), impulse });
	mPendingDamage.push_back({ inBody2.GetID(), impulse });
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
}

// ---------------------------------------------------------------------------
// SpawnFracture
// ---------------------------------------------------------------------------

void DestructibleTest::SpawnFracture(BodyID inPanelID)
{
	auto it = mFractureData.find(inPanelID);
	if (it == mFractureData.end())
		return;

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
		bcs.mLinearDamping                = 0.8f;
		bcs.mAngularDamping               = 0.8f;

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

	// Remove all constraints referencing this body via adjacency — O(degree).
	auto cleanByAdj = [&](bool inIsFrame)
	{
		UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adjMap = inIsFrame ? mFrameAdj : mPanelAdj;
		UnorderedMap<FixedConstraint *, int> &idxMap = inIsFrame ? mFrameIdx : mPanelIdx;
		auto adjIt = adjMap.find(inPanelID);
		if (adjIt == adjMap.end()) return;
		Array<Ref<FixedConstraint>> toRemove = adjIt->second;
		for (auto &cref : toRemove)
		{
			FixedConstraint *c = cref.GetPtr();
			BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();
			mBodyInterface->ActivateBody(id1 == inPanelID ? id2 : id1);
			auto posIt = idxMap.find(c);
			if (posIt != idxMap.end())
				UntrackConstraint(inIsFrame, posIt->second);
		}
	};
	cleanByAdj(false);
	cleanByAdj(true);

	mBodyInterface->RemoveBody(inPanelID);
	mBodyInterface->DestroyBody(inPanelID);
	mFractureData.erase(it);
	mChunkDamage.erase(inPanelID);
}

// ---------------------------------------------------------------------------
// CheckStructuralIntegrity — BFS from ground; detach unsupported chunks
// ---------------------------------------------------------------------------

void DestructibleTest::CheckStructuralIntegrity()
{
	if (mFrameConstraints.empty())
		return;

	UnorderedMap<BodyID, Array<int>> adj;
	for (int i = 0; i < (int)mFrameConstraints.size(); ++i)
	{
		FixedConstraint *c = mFrameConstraints[i];
		adj[c->GetBody1()->GetID()].push_back(i);
		adj[c->GetBody2()->GetID()].push_back(i);
	}

	UnorderedSet<BodyID> reachable;
	Array<BodyID> queue;

	for (auto &kv : adj)
		if (mBodyInterface->GetMotionType(kv.first) == EMotionType::Static)
			if (reachable.insert(kv.first).second)
				queue.push_back(kv.first);

	for (int qi = 0; qi < (int)queue.size(); ++qi)
	{
		BodyID current = queue[qi];
		auto it = adj.find(current);
		if (it == adj.end()) continue;
		for (int ci : it->second)
		{
			FixedConstraint *c = mFrameConstraints[ci];
			BodyID other = (c->GetBody1()->GetID() == current) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
			if (reachable.insert(other).second)
				queue.push_back(other);
		}
	}

	UnorderedSet<BodyID> unsupported;
	for (auto &kv : adj)
		if (reachable.find(kv.first) == reachable.end())
			unsupported.insert(kv.first);

	if (unsupported.empty())
		return;

	for (bool isFrame : { true, false })
	{
		Array<Ref<FixedConstraint>> &arr = isFrame ? mFrameConstraints : mPanelConstraints;
		for (int i = 0; i < (int)arr.size(); )
		{
			FixedConstraint *c = arr[i].GetPtr();
			if (unsupported.find(c->GetBody1()->GetID()) != unsupported.end() || unsupported.find(c->GetBody2()->GetID()) != unsupported.end())
				UntrackConstraint(isFrame, i);
			else
				++i;
		}
	}

	for (BodyID id : unsupported)
		mBodyInterface->ActivateBody(id);
}

// ---------------------------------------------------------------------------
// PrePhysicsUpdate
// ---------------------------------------------------------------------------

void DestructibleTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	// ---- Fire projectile ----
	if (mFire)
	{
		RVec3 start = inParams.mCameraState.mPos + RVec3(inParams.mCameraState.mForward);
		FireProjectile(start, inParams.mCameraState.mForward.Normalized());
		mFire = false;
	}

	// ---- Expire projectiles ----
	for (int i = (int)mProjectiles.size() - 1; i >= 0; --i)
	{
		mProjectiles[i].mLifeRemaining -= inParams.mDeltaTime;
		if (mProjectiles[i].mLifeRemaining <= 0.0f)
		{
			mBodyInterface->RemoveBody(mProjectiles[i].mID);
			mBodyInterface->DestroyBody(mProjectiles[i].mID);
			mProjectiles.erase(mProjectiles.begin() + i);
		}
	}

	// ---- Sleep shards ----
	for (int i = (int)mShardBodies.size() - 1; i >= 0; --i)
	{
		if (!mBodyInterface->IsActive(mShardBodies[i]))
		{
			mBodyInterface->RemoveBody(mShardBodies[i]);
			mBodyInterface->DestroyBody(mShardBodies[i]);
			mShardBodies.erase(mShardBodies.begin() + i);
		}
	}

	if (mPhysicsSystem->GetNumActiveBodies(EBodyType::RigidBody) == 0)
		return;

	auto t0 = std::chrono::high_resolution_clock::now();

	// ---- Apply pending contact damage ----
	{
		std::lock_guard<std::mutex> lock(mDamageMutex);
		for (auto &[id, impulse] : mPendingDamage)
		{
			auto it = mChunkDamage.find(id);
			if (it != mChunkDamage.end())
				it->second += impulse;
		}
		mPendingDamage.clear();
	}

	// ---- Destroy chunks that exceeded their damage threshold ----
	{
		Array<BodyID> toDestroy;
		for (auto &[id, damage] : mChunkDamage)
		{
			auto fit = mFractureData.find(id);
			if (fit == mFractureData.end()) continue;
			float threshold = fit->second.mIsFrame ? sFrameBreakForce : sPanelBreakForce;
			if (damage >= threshold)
				toDestroy.push_back(id);
		}
		for (BodyID id : toDestroy)
		{
			if (mBreakLogCount < 50)
			{
				++mBreakLogCount;
				auto fit = mFractureData.find(id);
				bool isFrame = fit != mFractureData.end() && fit->second.mIsFrame;
				auto dmgIt = mChunkDamage.find(id);
				float dmg = dmgIt != mChunkDamage.end() ? dmgIt->second : 0.0f;
				RVec3 pos = mBodyInterface->GetCenterOfMassPosition(id);
				printf("[CHUNK DESTROYED #%02d] type=%s  damage=%.1f  pos=(%.1f,%.1f,%.1f)\n",
					mBreakLogCount, isFrame ? "FRAME" : "PANEL", dmg,
					(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ());
				if (mLogFile)
				{
					fprintf(mLogFile, "[CHUNK DESTROYED #%02d] type=%s  damage=%.1f  pos=(%.1f,%.1f,%.1f)\n",
						mBreakLogCount, isFrame ? "FRAME" : "PANEL", dmg,
						(float)pos.GetX(), (float)pos.GetY(), (float)pos.GetZ());
					fflush(mLogFile);
				}
			}
			SpawnFracture(id); // removes body, constraints, and mChunkDamage[id]
		}
	}

	// ---- Panel gap check: break panels pulled >5 cm from attachment ----
	{
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

	// ---- Isolation check: free frame elements with exactly 1 connection ----
	for (int i = 0; i < (int)mFrameConstraints.size(); )
	{
		FixedConstraint *c = mFrameConstraints[i].GetPtr();
		BodyID id1 = c->GetBody1()->GetID();
		BodyID id2 = c->GetBody2()->GetID();
		auto it1 = mFrameConnCount.find(id1);
		auto it2 = mFrameConnCount.find(id2);
		bool b1_iso = mBodyInterface->GetMotionType(id1) == EMotionType::Dynamic
			&& it1 != mFrameConnCount.end() && it1->second == 1;
		bool b2_iso = mBodyInterface->GetMotionType(id2) == EMotionType::Dynamic
			&& it2 != mFrameConnCount.end() && it2->second == 1;
		if (b1_iso || b2_iso)
		{
			mBodyInterface->ActivateBody(id1);
			mBodyInterface->ActivateBody(id2);
			UntrackConstraint(true, i);
		}
		else ++i;
	}

	// ---- Structural integrity: BFS from ground, detach floating chunks ----
	CheckStructuralIntegrity();

	// ---- Fracture bodies that have lost all their constraints ----
	if (!mFractureData.empty())
	{
		Array<BodyID> to_fracture;
		for (auto &kv : mFractureData)
		{
			const UnorderedMap<BodyID, int> &cnt = kv.second.mIsFrame ? mFrameConnCount : mPanelConnCount;
			if (cnt.find(kv.first) == cnt.end())
				to_fracture.push_back(kv.first);
		}
		static constexpr int cMaxFracturesPerFrame = 3;
		int budget = cMaxFracturesPerFrame;
		for (BodyID id : to_fracture)
		{
			if (budget-- <= 0) break;
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

	inUI->CreateSlider(inSubMenu, "Frame Break Force (N\xc2\xb7s)", sFrameBreakForce, 100.0f, 2000.0f, 50.0f,
		[](float inValue) { sFrameBreakForce = inValue; });

	inUI->CreateTextButton(inSubMenu, "Reset", [this]() { RestartTest(); });
}
