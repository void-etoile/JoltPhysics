// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

#include <Samples.h>

#include <Tests/General/DestructibleTest.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Core/UnorderedMap.h>
#include <Jolt/Core/UnorderedSet.h>
#include <chrono>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Constraints/FixedConstraint.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Application/DebugUI.h>
#include <Input/Keyboard.h>
#include <Layers.h>

// ---------------------------------------------------------------------------
// Voronoi fracture helpers
// ---------------------------------------------------------------------------

static constexpr int cFracturePieces = 6;

struct Pt2 { float x, y; };

// Sutherland-Hodgman clip: keep vertices where nx*x + ny*y <= d.
static Array<Pt2> sClipPolygon(const Array<Pt2> &inPoly, float inNX, float inNY, float inD)
{
	Array<Pt2> out;
	int n = (int)inPoly.size();
	for (int i = 0; i < n; ++i)
	{
		const Pt2 &a = inPoly[i];
		const Pt2 &b = inPoly[(i + 1) % n];
		float da = inNX * a.x + inNY * a.y - inD;
		float db = inNX * b.x + inNY * b.y - inD;
		if (da <= 0.0f) out.push_back(a);
		if ((da < 0.0f) != (db < 0.0f))
		{
			float t = da / (da - db);
			out.push_back({ a.x + t * (b.x - a.x), a.y + t * (b.y - a.y) });
		}
	}
	return out;
}

// Generate Voronoi fracture geometry for a box panel with given half-extents.
// The thinnest axis becomes the extrusion direction; the other two form the 2D Voronoi plane.
// All output shape vertices are pre-centred so each body's origin is its cell centroid.
static void sGenerateFractureShapes(Vec3 inHalfExtents, uint32 inSeed, int inNumPieces,
	Array<RefConst<Shape>> &outShapes, Array<Vec3> &outLocalCenters)
{
	// Find thinnest axis
	int   thin_axis = 0;
	float min_he    = inHalfExtents.GetX();
	if (inHalfExtents.GetY() < min_he) { min_he = inHalfExtents.GetY(); thin_axis = 1; }
	if (inHalfExtents.GetZ() < min_he) { min_he = inHalfExtents.GetZ(); thin_axis = 2; }

	int ax0 = (thin_axis == 0) ? 1 : 0;
	int ax1 = (thin_axis == 2) ? 1 : 2;
	float ht = min_he;
	auto getHE = [&](int a) -> float {
		return a == 0 ? inHalfExtents.GetX() : (a == 1 ? inHalfExtents.GetY() : inHalfExtents.GetZ());
	};
	float h0 = getHE(ax0);
	float h1 = getHE(ax1);

	// Deterministic LCG seeded from position hash
	uint32 rng = inSeed;
	auto nextf = [&]() -> float {
		rng = rng * 1664525u + 1013904223u;
		return float(rng >> 8) / float(1u << 24);
	};

	// Seed points scattered across the panel face
	Array<Pt2> seeds(inNumPieces);
	for (Pt2 &s : seeds)
		s = { nextf() * 2.0f * h0 - h0, nextf() * 2.0f * h1 - h1 };

	// Bounding rectangle — starting polygon for Sutherland-Hodgman
	Array<Pt2> bbox;
	bbox.push_back(Pt2{ -h0, -h1 });
	bbox.push_back(Pt2{  h0, -h1 });
	bbox.push_back(Pt2{  h0,  h1 });
	bbox.push_back(Pt2{ -h0,  h1 });

	for (int i = 0; i < inNumPieces; ++i)
	{
		Array<Pt2> cell = bbox;
		for (int j = 0; j < inNumPieces && !cell.empty(); ++j)
		{
			if (j == i) continue;
			// Perpendicular bisector between seed i and seed j; keep the side containing seed i.
			float nx = seeds[j].x - seeds[i].x;
			float ny = seeds[j].y - seeds[i].y;
			float mx = (seeds[i].x + seeds[j].x) * 0.5f;
			float my = (seeds[i].y + seeds[j].y) * 0.5f;
			cell = sClipPolygon(cell, nx, ny, nx * mx + ny * my);
		}
		if ((int)cell.size() < 3) continue;

		// Centroid of the 2D Voronoi cell
		float cx = 0.0f, cy = 0.0f;
		for (const Pt2 &p : cell) { cx += p.x; cy += p.y; }
		cx /= float(cell.size()); cy /= float(cell.size());

		// 3D convex hull vertices: centred in the panel plane, extruded ±ht along thin axis
		Array<Vec3> verts;
		verts.reserve(cell.size() * 2);
		for (const Pt2 &p : cell)
		{
			float v[3] = { 0.0f, 0.0f, 0.0f };
			float w[3] = { 0.0f, 0.0f, 0.0f };
			v[ax0] = p.x - cx;  v[ax1] = p.y - cy;  v[thin_axis] = -ht;
			w[ax0] = p.x - cx;  w[ax1] = p.y - cy;  w[thin_axis] = +ht;
			verts.push_back(Vec3(v[0], v[1], v[2]));
			verts.push_back(Vec3(w[0], w[1], w[2]));
		}

		auto result = ConvexHullShapeSettings(verts.data(), (int)verts.size(), 0.0f).Create();
		if (!result.IsValid()) continue;

		float lc[3] = { 0.0f, 0.0f, 0.0f };
		lc[ax0] = cx;  lc[ax1] = cy;  lc[thin_axis] = 0.0f;
		outShapes.push_back(result.Get());
		outLocalCenters.push_back(Vec3(lc[0], lc[1], lc[2]));
	}
}

JPH_IMPLEMENT_RTTI_VIRTUAL(DestructibleTest)
{
	JPH_ADD_BASE_CLASS(DestructibleTest, Test)
}

void DestructibleTest::TrackConstraint(bool inIsFrame, Body *inA, Body *inB)
{
	FixedConstraintSettings s;
	s.mAutoDetectPoint = true;
	Ref<FixedConstraint> c = StaticCast<FixedConstraint>(s.Create(*inA, *inB));
	mPhysicsSystem->AddConstraint(c);

	Array<Ref<FixedConstraint>> &arr           = inIsFrame ? mFrameConstraints : mPanelConstraints;
	UnorderedMap<BodyID, int> &cnt             = inIsFrame ? mFrameConnCount   : mPanelConnCount;
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adj = inIsFrame ? mFrameAdj : mPanelAdj;
	UnorderedMap<FixedConstraint *, int> &idx  = inIsFrame ? mFrameIdx         : mPanelIdx;

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
	Array<Ref<FixedConstraint>> &arr           = inIsFrame ? mFrameConstraints : mPanelConstraints;
	UnorderedMap<BodyID, int> &cnt             = inIsFrame ? mFrameConnCount   : mPanelConnCount;
	UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adj = inIsFrame ? mFrameAdj : mPanelAdj;
	UnorderedMap<FixedConstraint *, int> &idx  = inIsFrame ? mFrameIdx         : mPanelIdx;

	FixedConstraint *c = arr[inPos].GetPtr();
	BodyID id1 = c->GetBody1()->GetID(), id2 = c->GetBody2()->GetID();

	// Decrement connection counts.
	if (--cnt[id1] == 0) cnt.erase(id1);
	if (--cnt[id2] == 0) cnt.erase(id2);

	// Remove from per-body adjacency lists (swap-and-pop by pointer identity).
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

	// Remove from index map and physics system.
	idx.erase(c);
	mPhysicsSystem->RemoveConstraint(c);

	// Swap-and-pop from main array; update index of the displaced element.
	int last = (int)arr.size() - 1;
	if (inPos != last)
	{
		arr[inPos] = std::move(arr[last]);
		idx[arr[inPos].GetPtr()] = inPos;
	}
	arr.pop_back();
}

float DestructibleTest::sPanelBreakForce	=   50.0f;
float DestructibleTest::sFrameBreakForce	=  500.0f;

// Building layout
static constexpr int	cNumBays		= 3;
static constexpr int	cNumFloors		= 3;
static constexpr float	cBayWidth		= 2.0f;
static constexpr float	cFloorHeight	= 2.0f;
static constexpr float	cStubHeight		= 0.5f;		// static ground-attached column stub height

// Structural element half-extents
static const Vec3 cStubHalf		(0.1f,  cStubHeight * 0.5f,             0.1f);	// indestructible base stub
static const Vec3 cColumnHalf	(0.1f,  cFloorHeight * 0.5f,            0.1f);	// upper column segment
static const Vec3 cBeamHalf		(cBayWidth * 0.5f - 0.1f, 0.1f,        0.1f);	// horizontal beam spanning a bay
static const Vec3 cPanelHalf	(cBayWidth * 0.5f - 0.2f, cFloorHeight * 0.5f - 0.2f, 0.05f); // infill panel

void DestructibleTest::Initialize()
{
	mPanelConstraints.clear();
	mFrameConstraints.clear();
	mFractureData.clear();
	mShardBodies.clear();
	mFire = false;
	mWasFire = false;
	mFrameConnCount.clear();
	mPanelConnCount.clear();
	mFrameAdj.clear();
	mPanelAdj.clear();
	mFrameIdx.clear();
	mPanelIdx.clear();

	CreateFloor();

	// All building pieces share one subgroup so they never collide with each other.
	// (GroupFilterTable: same SubGroupID → CanCollide returns false by definition.)
	Ref<GroupFilterTable> building_filter = new GroupFilterTable(1);

	RefConst<Shape> stub_shape	= new BoxShape(cStubHalf);
	RefConst<Shape> col_shape	= new BoxShape(cColumnHalf);
	RefConst<Shape> beam_shape	= new BoxShape(cBeamHalf);
	RefConst<Shape> panel_shape	= new BoxShape(cPanelHalf);

	// Column x positions, symmetrically centred on the origin
	float col_x[cNumBays + 1];
	for (int j = 0; j <= cNumBays; ++j)
		col_x[j] = (j - cNumBays * 0.5f) * cBayWidth;

	// Helper that assigns the shared group filter and adds the body.
	// All structural bodies start sleeping; they wake when struck by a projectile.
	auto addBody = [&](const BodyCreationSettings &inBCS, EActivation) -> Body *
	{
		Body *b = mBodyInterface->CreateBody(inBCS);
		b->SetCollisionGroup(CollisionGroup(building_filter, 0, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};


	// -----------------------------------------------------------------------
	// Columns: one segment per floor per column position.
	//   Base segments are Static — they're the indestructible foundation.
	//   Upper segments are Dynamic and held by frame constraints.
	// -----------------------------------------------------------------------
	Body *col[cNumBays + 1][cNumFloors];
	for (int j = 0; j <= cNumBays; ++j)
		for (int i = 0; i < cNumFloors; ++i)
		{
			bool is_base = (i == 0);
			// Stubs sit at ground level; upper columns stack above them.
			float y = is_base ? cStubHeight * 0.5f
			                  : cStubHeight + (i - 0.5f) * cFloorHeight;

			BodyCreationSettings bcs(is_base ? stub_shape : col_shape,
				RVec3(col_x[j], y, 0.0f), Quat::sIdentity(),
				is_base ? EMotionType::Static  : EMotionType::Dynamic,
				is_base ? Layers::NON_MOVING   : Layers::STRUCTURE);

			if (!is_base)
			{
				bcs.mOverrideMassProperties			= EOverrideMassProperties::CalculateInertia;
				bcs.mMassPropertiesOverride.mMass	= 200.0f;
			}

			col[j][i] = addBody(bcs, is_base ? EActivation::DontActivate : EActivation::Activate);

			if (!is_base)
			{
				uint32 seed = uint32(int(col_x[j] * 100.0f) * 73856093u ^ int(y * 100.0f) * 19349663u ^ 31337u);
				FractureInfo info;
				info.mIsFrame = true;
				sGenerateFractureShapes(cColumnHalf, seed, 4, info.mShapes, info.mLocalCenters);
				mFractureData[col[j][i]->GetID()] = info;
			}
		}

	// -----------------------------------------------------------------------
	// Beams: one per bay per floor, positioned at the top of each floor
	// -----------------------------------------------------------------------
	Body *beam[cNumBays][cNumFloors];
	for (int j = 0; j < cNumBays; ++j)
		for (int i = 0; i < cNumFloors; ++i)
		{
			float x = (col_x[j] + col_x[j + 1]) * 0.5f;
			float y = cStubHeight + i * cFloorHeight;

			BodyCreationSettings bcs(beam_shape, RVec3(x, y, 0.0f), Quat::sIdentity(),
				EMotionType::Dynamic, Layers::STRUCTURE);
			bcs.mOverrideMassProperties			= EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass	= 100.0f;

			beam[j][i] = addBody(bcs, EActivation::Activate);

			{
				uint32 seed = uint32(int(x * 100.0f) * 73856093u ^ int(y * 100.0f) * 19349663u ^ 12345u);
				FractureInfo info;
				info.mIsFrame = true;
				sGenerateFractureShapes(cBeamHalf, seed, 4, info.mShapes, info.mLocalCenters);
				mFractureData[beam[j][i]->GetID()] = info;
			}
		}

	// -----------------------------------------------------------------------
	// Infill panels: one per bay per floor, centred in the bay opening.
	// No panel at stub level (i == 0) — the 50 cm bay is too short.
	// -----------------------------------------------------------------------
	Body *panel[cNumBays][cNumFloors];
	for (int j = 0; j < cNumBays; ++j)
		for (int i = 1; i < cNumFloors; ++i)
		{
			float x = (col_x[j] + col_x[j + 1]) * 0.5f;
			float y = cStubHeight + (i - 0.5f) * cFloorHeight;

			BodyCreationSettings bcs(panel_shape, RVec3(x, y, 0.0f), Quat::sIdentity(),
				EMotionType::Dynamic, Layers::STRUCTURE);
			bcs.mOverrideMassProperties			= EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass	= 20.0f;

			panel[j][i] = addBody(bcs, EActivation::Activate);

			// Register fracture geometry (deterministic seed from world position)
			uint32 seed = uint32(int(x * 100.0f) * 73856093u ^ int(y * 100.0f) * 19349663u);
			FractureInfo info;
			sGenerateFractureShapes(cPanelHalf, seed, cFracturePieces, info.mShapes, info.mLocalCenters);
			mFractureData[panel[j][i]->GetID()] = info;
		}

	// -----------------------------------------------------------------------
	// Frame constraints (high break force)
	//   • Vertical: connect column segment i to segment i+1 within each column
	//   • Horizontal: connect each beam to the column segments flanking it
	// -----------------------------------------------------------------------
	for (int j = 0; j <= cNumBays; ++j)
		for (int i = 0; i + 1 < cNumFloors; ++i)
			TrackConstraint(true, col[j][i], col[j][i + 1]);

	for (int j = 0; j < cNumBays; ++j)
		for (int i = 0; i < cNumFloors; ++i)
		{
			TrackConstraint(true, col[j][i],     beam[j][i]);
			TrackConstraint(true, col[j + 1][i], beam[j][i]);
		}

	// -----------------------------------------------------------------------
	// Panel constraints (low break force)
	//   Each panel is pinned to the column segments on its left and right.
	//   Weight is transferred through the columns into the static base.
	// -----------------------------------------------------------------------
	for (int j = 0; j < cNumBays; ++j)
		for (int i = 1; i < cNumFloors; ++i)	// skip stub level — no panel there
		{
			TrackConstraint(false, col[j][i],     panel[j][i]);
			TrackConstraint(false, col[j + 1][i], panel[j][i]);
		}

	// 30 houses in a 5-column × 6-row grid, offset from the main wall.
	// Each house is 6 m wide (X) × 4 m deep (Z); 9 m × 7 m slots give 3 m clearance.
	static constexpr int   cHouseCols      = 5;
	static constexpr int   cHouseRows      = 6;
	static constexpr float cHouseSpacingX  = 9.0f;
	static constexpr float cHouseSpacingZ  = 7.0f;
	const float house_start_x = 12.0f;
	const float house_start_z = -(cHouseRows - 1) * cHouseSpacingZ * 0.5f;

	for (int row = 0; row < cHouseRows; ++row)
		for (int col = 0; col < cHouseCols; ++col)
			BuildHouse(RVec3(house_start_x + col * cHouseSpacingX,
				0.0f,
				house_start_z + row * cHouseSpacingZ));

	mInitialPanelCount = (int)mPanelConstraints.size();
	mInitialFrameCount = (int)mFrameConstraints.size();
	mLastBreakCheckUs  = 0.0f;
}

void DestructibleTest::BuildHouse(RVec3Arg inCenter)
{
	// House dimensions
	static constexpr float hW    = 3.0f;  // half-width along X (6 m total)
	static constexpr float hD    = 2.0f;  // half-depth along Z (4 m total)
	static constexpr float hH    = 3.0f;  // wall height
	static constexpr float hRise = 1.0f;  // roof rise above wall top → ridge at y = hH + hRise

	// Roof geometry: slope from eave (z = ±hD, y = hH) to ridge (z = 0, y = hH + hRise)
	const float slope_half = Sqrt(hD * hD + hRise * hRise) * 0.5f; // half-length along the slope
	const float roof_angle = atan2f(hRise, hD);                     // tilt from horizontal

	// Stub + pillar split: 50 cm static stub at ground level, dynamic pillar above
	static constexpr float hStub     = 0.5f;
	const float stub_half_h          = hStub * 0.5f;                   // 0.25 m
	const float pillar_half_h        = (hH - hStub) * 0.5f;           // 1.25 m
	const float stub_cy              = stub_half_h;                    // 0.25 m
	const float col_cy               = hStub + pillar_half_h;         // 1.75 m
	const float panel_half_h         = pillar_half_h - 0.1f;          // 1.15 m (with clearance)

	// Shape library
	RefConst<Shape> s_stub  = new BoxShape(Vec3(0.1f, stub_half_h,  0.1f));
	RefConst<Shape> s_col   = new BoxShape(Vec3(0.1f, pillar_half_h, 0.1f));
	RefConst<Shape> s_bx    = new BoxShape(Vec3(hW * 0.5f - 0.1f,  0.1f,          0.1f));          // X-axis wall plate
	RefConst<Shape> s_bz    = new BoxShape(Vec3(0.1f,               0.1f, hD * 0.5f - 0.1f));     // Z-axis wall plate
	RefConst<Shape> s_ridge = new BoxShape(Vec3(hW - 0.1f,          0.1f,          0.1f));          // ridge beam
	RefConst<Shape> s_roof  = new BoxShape(Vec3(hW - 0.1f,         0.05f,      slope_half));       // roof panel (local, before tilt)
	RefConst<Shape> s_pwf   = new BoxShape(Vec3(hW * 0.5f - 0.2f, panel_half_h,          0.05f)); // front/back wall panel
	RefConst<Shape> s_pws   = new BoxShape(Vec3(0.05f,             panel_half_h, hD * 0.5f - 0.15f)); // side wall panel

	// House pieces don't collide with each other, but can collide with the separate wall structure
	Ref<GroupFilterTable> house_filter = new GroupFilterTable(1);

	// Create a body at inCenter + localPos with the given shape and motion type.
	// All house bodies start sleeping; they wake when struck by a projectile.
	auto addBody = [&](RVec3Arg inLocalPos, QuatArg inRot, RefConst<Shape> inShape,
		EMotionType inMotion, ObjectLayer inLayer, float inMass, EActivation) -> Body *
	{
		BodyCreationSettings bcs(inShape, inCenter + inLocalPos, inRot, inMotion, inLayer);
		if (inMotion != EMotionType::Static)
		{
			bcs.mOverrideMassProperties			= EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass	= inMass;
		}
		Body *b = mBodyInterface->CreateBody(bcs);
		b->SetCollisionGroup(CollisionGroup(house_filter, 0, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};

	auto frameC = [&](Body *inA, Body *inB) { TrackConstraint(true,  inA, inB); };
	auto panelC = [&](Body *inA, Body *inB) { TrackConstraint(false, inA, inB); };

	auto registerHouseElement = [&](Body *b, float lx, float ly, float lz, Vec3 inHalfExtents, int inNumPieces, bool inIsFrame)
	{
		RVec3 wp = inCenter + RVec3(lx, ly, lz);
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = inIsFrame;
		sGenerateFractureShapes(inHalfExtents, seed, inNumPieces, info.mShapes, info.mLocalCenters);
		mFractureData[b->GetID()] = info;
	};

	auto registerHousePanel = [&](Body *b, float lx, float ly, float lz, Vec3 inHalfExtents)
	{
		registerHouseElement(b, lx, ly, lz, inHalfExtents, cFracturePieces, false);
	};

	// -----------------------------------------------------------------------
	// Columns: 50 cm static stub at base + dynamic pillar above, one per corner.
	//   Stub is indestructible; pillar can be knocked free via frame constraints.
	//
	//   front (z=-hD):  col_FL --- col_FM --- col_FR
	//   mids  (z=0):    col_ML              col_MR
	//   back  (z=+hD):  col_BL --- col_BM --- col_BR
	// -----------------------------------------------------------------------
	auto addColPair = [&](float x, float z) -> Body *
	{
		Body *stub   = addBody(RVec3(x, stub_cy, z), Quat::sIdentity(), s_stub,
			EMotionType::Static,  Layers::NON_MOVING, 0,     EActivation::DontActivate);
		Body *pillar = addBody(RVec3(x, col_cy,  z), Quat::sIdentity(), s_col,
			EMotionType::Dynamic, Layers::STRUCTURE,  300.0f, EActivation::Activate);
		frameC(stub, pillar);
		registerHouseElement(pillar, x, col_cy, z, Vec3(0.1f, pillar_half_h, 0.1f), 3, true);
		return pillar;
	};

	Body *col_FL = addColPair(-hW, -hD);
	Body *col_FM = addColPair(  0, -hD);
	Body *col_FR = addColPair(+hW, -hD);
	Body *col_ML = addColPair(-hW,   0);
	Body *col_MR = addColPair(+hW,   0);
	Body *col_BL = addColPair(-hW, +hD);
	Body *col_BM = addColPair(  0, +hD);
	Body *col_BR = addColPair(+hW, +hD);

	// -----------------------------------------------------------------------
	// Wall plates (8 dynamic beams): one per bay at y = hH (top of wall)
	// -----------------------------------------------------------------------
	const float bcy = hH;
	const float bcx = hW * 0.5f;  // bay centre offset along X
	const float bcz = hD * 0.5f;  // bay centre offset along Z

	// Front wall (z = -hD, beams along X)
	Body *bf1 = addBody(RVec3(-bcx, bcy, -hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	Body *bf2 = addBody(RVec3(+bcx, bcy, -hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	// Back wall (z = +hD, beams along X)
	Body *bb1 = addBody(RVec3(-bcx, bcy, +hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	Body *bb2 = addBody(RVec3(+bcx, bcy, +hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	// Left wall (x = -hW, beams along Z)
	Body *bl1 = addBody(RVec3(-hW, bcy, -bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	Body *bl2 = addBody(RVec3(-hW, bcy, +bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	// Right wall (x = +hW, beams along Z)
	Body *br1 = addBody(RVec3(+hW, bcy, -bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);
	Body *br2 = addBody(RVec3(+hW, bcy, +bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::STRUCTURE, 50, EActivation::Activate);

	// Register wall-plate beams as frame elements
	const Vec3 bx_he(hW * 0.5f - 0.1f, 0.1f, 0.1f);
	const Vec3 bz_he(0.1f, 0.1f, hD * 0.5f - 0.1f);
	registerHouseElement(bf1, -bcx, bcy, -hD, bx_he, 3, true);
	registerHouseElement(bf2, +bcx, bcy, -hD, bx_he, 3, true);
	registerHouseElement(bb1, -bcx, bcy, +hD, bx_he, 3, true);
	registerHouseElement(bb2, +bcx, bcy, +hD, bx_he, 3, true);
	registerHouseElement(bl1, -hW,  bcy, -bcz, bz_he, 3, true);
	registerHouseElement(bl2, -hW,  bcy, +bcz, bz_he, 3, true);
	registerHouseElement(br1, +hW,  bcy, -bcz, bz_he, 3, true);
	registerHouseElement(br2, +hW,  bcy, +bcz, bz_he, 3, true);

	// -----------------------------------------------------------------------
	// Ridge beam: centred above the house, at peak height
	// -----------------------------------------------------------------------
	Body *ridge = addBody(RVec3(0, hH + hRise, 0), Quat::sIdentity(), s_ridge, EMotionType::Dynamic, Layers::STRUCTURE, 80, EActivation::Activate);
	registerHouseElement(ridge, 0, hH + hRise, 0, Vec3(hW - 0.1f, 0.1f, 0.1f), 3, true);

	// -----------------------------------------------------------------------
	// Roof panels (2): tilted slabs whose local Z axis runs along the slope.
	//   Front panel: eave at (z=-hD, y=hH), ridge at (z=0, y=hH+hRise)
	//   Back panel:  eave at (z=+hD, y=hH), ridge at (z=0, y=hH+hRise)
	// Rotating about the X axis by -roof_angle tilts the front panel correctly
	// (verified: local ±Z ends land on eave/ridge world positions).
	// -----------------------------------------------------------------------
	Body *roof_f = addBody(RVec3(0, hH + hRise * 0.5f, -hD * 0.5f),
		Quat::sRotation(Vec3::sAxisX(), -roof_angle), s_roof,
		EMotionType::Dynamic, Layers::STRUCTURE, 60, EActivation::Activate);
	registerHouseElement(roof_f, 0, hH + hRise * 0.5f, -hD * 0.5f,
		Vec3(hW - 0.1f, 0.05f, slope_half), cFracturePieces, true);

	Body *roof_b = addBody(RVec3(0, hH + hRise * 0.5f, +hD * 0.5f),
		Quat::sRotation(Vec3::sAxisX(), +roof_angle), s_roof,
		EMotionType::Dynamic, Layers::STRUCTURE, 60, EActivation::Activate);
	registerHouseElement(roof_b, 0, hH + hRise * 0.5f, +hD * 0.5f,
		Vec3(hW - 0.1f, 0.05f, slope_half), cFracturePieces, true);

	// -----------------------------------------------------------------------
	// Wall infill panels (8 dynamic): 2 per wall face
	// -----------------------------------------------------------------------
	const Vec3 pwf_he(hW * 0.5f - 0.2f, panel_half_h, 0.05f);
	const Vec3 pws_he(0.05f, panel_half_h, hD * 0.5f - 0.15f);

	// Front wall (z = -hD, panel normal along Z)
	Body *fp1 = addBody(RVec3(-bcx, col_cy, -hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	Body *fp2 = addBody(RVec3(+bcx, col_cy, -hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	// Back wall (z = +hD)
	Body *bp1 = addBody(RVec3(-bcx, col_cy, +hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	Body *bp2 = addBody(RVec3(+bcx, col_cy, +hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	// Left wall (x = -hW, panel normal along X)
	Body *lp1 = addBody(RVec3(-hW, col_cy, -bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	Body *lp2 = addBody(RVec3(-hW, col_cy, +bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	// Right wall (x = +hW)
	Body *rp1 = addBody(RVec3(+hW, col_cy, -bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);
	Body *rp2 = addBody(RVec3(+hW, col_cy, +bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::STRUCTURE, 20, EActivation::Activate);

	registerHousePanel(fp1, -bcx, col_cy, -hD, pwf_he);
	registerHousePanel(fp2, +bcx, col_cy, -hD, pwf_he);
	registerHousePanel(bp1, -bcx, col_cy, +hD, pwf_he);
	registerHousePanel(bp2, +bcx, col_cy, +hD, pwf_he);
	registerHousePanel(lp1, -hW,  col_cy, -bcz, pws_he);
	registerHousePanel(lp2, -hW,  col_cy, +bcz, pws_he);
	registerHousePanel(rp1, +hW,  col_cy, -bcz, pws_he);
	registerHousePanel(rp2, +hW,  col_cy, +bcz, pws_he);

	// -----------------------------------------------------------------------
	// Frame constraints (high break force)
	// -----------------------------------------------------------------------
	// Columns → wall plate beams
	frameC(col_FL, bf1);  frameC(col_FM, bf1);   // front left bay
	frameC(col_FM, bf2);  frameC(col_FR, bf2);   // front right bay
	frameC(col_BL, bb1);  frameC(col_BM, bb1);   // back left bay
	frameC(col_BM, bb2);  frameC(col_BR, bb2);   // back right bay
	frameC(col_FL, bl1);  frameC(col_ML, bl1);   // left front bay
	frameC(col_ML, bl2);  frameC(col_BL, bl2);   // left back bay
	frameC(col_FR, br1);  frameC(col_MR, br1);   // right front bay
	frameC(col_MR, br2);  frameC(col_BR, br2);   // right back bay

	// Wall plate beams → ridge
	frameC(bf1, ridge);  frameC(bf2, ridge);
	frameC(bb1, ridge);  frameC(bb2, ridge);

	// Roof panels → wall plate beams (eave): high break force
	frameC(bf1, roof_f);  frameC(bf2, roof_f);
	frameC(bb1, roof_b);  frameC(bb2, roof_b);
	// Roof panels → ridge (peak): low break force so when the ridge loses its
	// direct wall-plate support and moves, these connections release quickly
	// and the ridge falls rather than floating suspended by the A-frame.
	panelC(ridge, roof_f);
	panelC(ridge, roof_b);

	// -----------------------------------------------------------------------
	// Panel constraints (low break force)
	// -----------------------------------------------------------------------
	panelC(col_FL, fp1);  panelC(col_FM, fp1);
	panelC(col_FM, fp2);  panelC(col_FR, fp2);
	panelC(col_BL, bp1);  panelC(col_BM, bp1);
	panelC(col_BM, bp2);  panelC(col_BR, bp2);
	panelC(col_FL, lp1);  panelC(col_ML, lp1);
	panelC(col_ML, lp2);  panelC(col_BL, lp2);
	panelC(col_FR, rp1);  panelC(col_MR, rp1);
	panelC(col_MR, rp2);  panelC(col_BR, rp2);
}

void DestructibleTest::ProcessInput(const ProcessInputParams &inParams)
{
	mFire = inParams.mKeyboard->IsKeyPressedAndTriggered(EKey::Return, mWasFire);
}

void DestructibleTest::SaveInputState(StateRecorder &inStream) const
{
	inStream.Write(mFire);
}

void DestructibleTest::RestoreInputState(StateRecorder &inStream)
{
	inStream.Read(mFire);
}

void DestructibleTest::FireProjectile(RVec3Arg inPos, Vec3Arg inDirection)
{
	static constexpr float cProjectileSpeed = 20.0f;

	BodyCreationSettings proj(new SphereShape(0.25f),
		inPos, Quat::sIdentity(),
		EMotionType::Dynamic, Layers::MOVING);
	proj.mOverrideMassProperties		= EOverrideMassProperties::CalculateInertia;
	proj.mMassPropertiesOverride.mMass	= 50.0f;
	proj.mLinearVelocity				= inDirection.Normalized() * cProjectileSpeed;

	BodyID id = mBodyInterface->CreateAndAddBody(proj, EActivation::Activate);
	mProjectiles.push_back({ id, 3.0f });
}

void DestructibleTest::PrePhysicsUpdate(const PreUpdateParams &inParams)
{
	if (mFire)
	{
		// Offset the spawn point one metre in front of the camera so the sphere doesn't
		// start overlapping the camera's near plane or any geometry behind the view.
		RVec3 start = inParams.mCameraState.mPos + RVec3(inParams.mCameraState.mForward);
		FireProjectile(start, inParams.mCameraState.mForward.Normalized());
		mFire = false;
	}

	// Expire projectiles after their lifetime elapses.
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

	// Remove shards that have gone to sleep — they're done contributing to the scene
	// and freeing them keeps the broadphase and body pool lean.
	for (int i = (int)mShardBodies.size() - 1; i >= 0; --i)
	{
		if (!mBodyInterface->IsActive(mShardBodies[i]))
		{
			mBodyInterface->RemoveBody(mShardBodies[i]);
			mBodyInterface->DestroyBody(mShardBodies[i]);
			mShardBodies.erase(mShardBodies.begin() + i);
		}
	}

	// No active bodies → nothing can break or fracture this frame.
	if (mPhysicsSystem->GetNumActiveBodies(EBodyType::RigidBody) == 0)
		return;

	// extra[id] accumulates the impulse from constraints that already broke this
	// frame and shared body `id`. When checking surviving constraints we add this
	// redistribution so that a neighbour that was near the threshold gets pushed
	// over it — modelling the load transfer that occurs when a support is removed.
	UnorderedMap<BodyID, float> extra;

	// Scale to convert angular impulse (N·m·s) → linear impulse (N·s).
	// Assumes a ~1 m lever arm; larger = more sensitive to rotational stress.
	static constexpr float cAngularLambdaScale = 1.0f;

	// breakConstraints: break a constraint when either
	//   (a) combined linear+angular stress exceeds inThreshold, OR
	//   (b) the world-space gap between attachment points exceeds inMaxGap.
	auto breakConstraints = [&](bool inIsFrame, float inThreshold, float inMaxGap)
	{
		Array<Ref<FixedConstraint>> &inConstraints = inIsFrame ? mFrameConstraints : mPanelConstraints;
		// Forward iteration so earlier breaks cascade into later checks this frame.
		for (int i = 0; i < (int)inConstraints.size(); )
		{
			FixedConstraint *c = inConstraints[i].GetPtr();
			Body *body1 = c->GetBody1();
			Body *body2 = c->GetBody2();
			BodyID id1 = body1->GetID();
			BodyID id2 = body2->GetID();

			// Both bodies asleep → no stress accumulates; skip expensive gap calc.
			if (!body1->IsActive() && !body2->IsActive())
			{
				++i;
				continue;
			}

			// Combined stress: linear impulse + angular impulse scaled to linear units.
			float own = c->GetTotalLambdaPosition().Length()
				+ c->GetTotalLambdaRotation().Length() * cAngularLambdaScale;
			float lambda = own;

			auto it1 = extra.find(id1);  if (it1 != extra.end()) lambda += it1->second;
			auto it2 = extra.find(id2);  if (it2 != extra.end()) lambda += it2->second;

			// Gap check: measure separation between the two attachment points in world space.
			// GetConstraintToBody*Matrix().GetTranslation() is the anchor offset in body-local space.
			bool gap_exceeded = false;
			{
				Vec3 local1 = c->GetConstraintToBody1Matrix().GetTranslation();
				Vec3 local2 = c->GetConstraintToBody2Matrix().GetTranslation();
				RVec3 p1 = body1->GetCenterOfMassPosition() + RVec3(body1->GetRotation() * local1);
				RVec3 p2 = body2->GetCenterOfMassPosition() + RVec3(body2->GetRotation() * local2);
				gap_exceeded = float((p2 - p1).LengthSq()) > inMaxGap * inMaxGap;
			}

			if (lambda > inThreshold || gap_exceeded)
			{
				auto addExtra = [&](BodyID inID)
				{
					auto it = extra.find(inID);
					if (it != extra.end()) it->second += own;
					else extra.insert({ inID, own });
				};
				addExtra(id1);
				addExtra(id2);

				mBodyInterface->ActivateBody(id1);
				mBodyInterface->ActivateBody(id2);
				UntrackConstraint(inIsFrame, i);
				// don't increment — swap-and-pop means position i now holds the former last element
			}
			else
			{
				++i;
			}
		}
	};

	auto t0 = std::chrono::high_resolution_clock::now();
	breakConstraints(false, sPanelBreakForce, 0.05f);	// 5 cm gap breaks panels
	breakConstraints(true,  sFrameBreakForce, 0.10f);	// 10 cm gap breaks frame

	// A dynamic body left with exactly one frame constraint pivots around that
	// single anchor and oscillates with large amplitude (Jolt's velocity-based
	// solver can't hold a single-point attachment rigidly). Break the last
	// connection and let the body fall freely instead.
	// mFrameConnCount is already up-to-date from the breakConstraints pass above.
	for (int i = 0; i < (int)mFrameConstraints.size(); )
	{
		FixedConstraint *c = mFrameConstraints[i].GetPtr();
		BodyID id1 = c->GetBody1()->GetID();
		BodyID id2 = c->GetBody2()->GetID();
		auto it1 = mFrameConnCount.find(id1);
		auto it2 = mFrameConnCount.find(id2);
		bool b1_isolated = mBodyInterface->GetMotionType(id1) == EMotionType::Dynamic
			&& it1 != mFrameConnCount.end() && it1->second == 1;
		bool b2_isolated = mBodyInterface->GetMotionType(id2) == EMotionType::Dynamic
			&& it2 != mFrameConnCount.end() && it2->second == 1;
		if (b1_isolated || b2_isolated)
		{
			mBodyInterface->ActivateBody(id1);
			mBodyInterface->ActivateBody(id2);
			UntrackConstraint(true, i);
			// don't increment — swap-and-pop; position i now holds the former last element
		}
		else
		{
			++i;
		}
	}

	// Fracture registered elements that have lost all their relevant constraints.
	// Frame elements fracture when no frame constraints remain; panel elements when no panel constraints remain.
	// mFrameConnCount/mPanelConnCount give O(1) per-body lookup: if a body ID is absent, its count is 0.
	if (!mFractureData.empty())
	{
		Array<BodyID> to_fracture;
		for (auto &kv : mFractureData)
		{
			const UnorderedMap<BodyID, int> &cnt = kv.second.mIsFrame ? mFrameConnCount : mPanelConnCount;
			if (cnt.find(kv.first) == cnt.end())
				to_fracture.push_back(kv.first);
		}
		// Limit fracture events per frame so the broadphase rebuild and new-body
		// simulation cost is spread over several frames rather than spiking in one.
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

void DestructibleTest::SpawnFracture(BodyID inPanelID)
{
	auto it = mFractureData.find(inPanelID);
	if (it == mFractureData.end())
		return;

	FractureInfo &info    = it->second;
	RVec3 panel_pos       = mBodyInterface->GetCenterOfMassPosition(inPanelID);
	Quat  panel_rot       = mBodyInterface->GetRotation(inPanelID);
	Vec3  panel_vel       = mBodyInterface->GetLinearVelocity(inPanelID);
	Vec3  panel_ang       = mBodyInterface->GetAngularVelocity(inPanelID);

	// Hard cap: if already at the shard limit, skip spawning to keep simulation bounded.
	static constexpr int cMaxShards = 300;
	const int slots = cMaxShards - (int)mShardBodies.size();

	// Create all shard bodies first, then batch-insert into the broadphase.
	// Use Layers::DEBRIS so shards only collide with static geometry (NON_MOVING),
	// eliminating all shard-shard and shard-vs-structure GJK work.
	Array<BodyID> shard_ids;
	shard_ids.reserve(info.mShapes.size());

	for (int i = 0; i < (int)info.mShapes.size() && i < slots; ++i)
	{
		RVec3 frag_pos = panel_pos + RVec3(panel_rot * info.mLocalCenters[i]);

		BodyCreationSettings bcs(info.mShapes[i], frag_pos, panel_rot,
			EMotionType::Dynamic, Layers::DEBRIS);
		bcs.mOverrideMassProperties        = EOverrideMassProperties::CalculateInertia;
		bcs.mMassPropertiesOverride.mMass  = 3.5f;
		bcs.mLinearVelocity                = panel_vel;
		bcs.mAngularVelocity               = panel_ang;
		bcs.mLinearDamping                 = 0.8f;
		bcs.mAngularDamping                = 0.8f;

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

	// Remove lingering constraints referencing this body via the adjacency index — O(degree)
	// rather than O(total_constraints). We copy the list since UntrackConstraint mutates it.
	auto cleanByAdj = [&](bool inIsFrame)
	{
		UnorderedMap<BodyID, Array<Ref<FixedConstraint>>> &adjMap = inIsFrame ? mFrameAdj : mPanelAdj;
		UnorderedMap<FixedConstraint *, int> &idxMap = inIsFrame ? mFrameIdx : mPanelIdx;
		auto adjIt = adjMap.find(inPanelID);
		if (adjIt == adjMap.end()) return;
		Array<Ref<FixedConstraint>> toRemove = adjIt->second;  // copy — UntrackConstraint mutates adjMap
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
}

void DestructibleTest::CheckStructuralIntegrity()
{
	if (mFrameConstraints.empty())
		return;

	// Build adjacency: body → indices of frame constraints touching it.
	// We only traverse frame constraints — panels don't provide structural support.
	UnorderedMap<BodyID, Array<int>> adj;
	for (int i = 0; i < (int)mFrameConstraints.size(); ++i)
	{
		FixedConstraint *c = mFrameConstraints[i];
		adj[c->GetBody1()->GetID()].push_back(i);
		adj[c->GetBody2()->GetID()].push_back(i);
	}

	// BFS from all static bodies in the frame graph.
	// Static bodies are the foundation — they're always ground-connected.
	UnorderedSet<BodyID> reachable;
	Array<BodyID> queue;

	for (auto &kv : adj)
	{
		if (mBodyInterface->GetMotionType(kv.first) == EMotionType::Static)
		{
			if (reachable.insert(kv.first).second)
				queue.push_back(kv.first);
		}
	}

	for (int qi = 0; qi < (int)queue.size(); ++qi)
	{
		BodyID current = queue[qi];
		auto it = adj.find(current);
		if (it == adj.end())
			continue;
		for (int ci : it->second)
		{
			FixedConstraint *c = mFrameConstraints[ci];
			BodyID other = (c->GetBody1()->GetID() == current) ? c->GetBody2()->GetID() : c->GetBody1()->GetID();
			if (reachable.insert(other).second)
				queue.push_back(other);
		}
	}

	// Collect bodies in the frame graph that are NOT reachable from the ground.
	UnorderedSet<BodyID> unsupported;
	for (auto &kv : adj)
		if (reachable.find(kv.first) == reachable.end())
			unsupported.insert(kv.first);

	if (unsupported.empty())
		return;

	// Detach: remove every frame and panel constraint touching an unsupported body.
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

String DestructibleTest::GetStatusString() const
{
	char buf[128];
	snprintf(buf, sizeof(buf),
		"Panels: %d / %d  |  Frame: %d / %d  |  Break check: %.1f us",
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
