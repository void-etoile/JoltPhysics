// Jolt Physics Library (https://github.com/jrouwe/JoltPhysics)
// SPDX-FileCopyrightText: 2021 Jorrit Rouwe
// SPDX-License-Identifier: MIT

// Building construction code for DestructibleTest.
// Contains: Voronoi fracture helpers, layout constants, and all Build* methods.
// Kept separate so the simulation logic in DestructibleTest.cpp stays readable.

#include <Samples.h>

#include <Tests/General/DestructibleTest.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/GroupFilterTable.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
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

// Like sGenerateFractureShapes but clips Voronoi cells to an arbitrary 2D polygon
// (XZ plane) rather than a bounding rectangle.  Used for non-rectangular bodies
// such as the octagonal tower roof.
static void sGenerateFractureShapesPoly(const Array<Pt2> &inBoundary, float inHalfThick,
	uint32 inSeed, int inNumPieces,
	Array<RefConst<Shape>> &outShapes, Array<Vec3> &outLocalCenters)
{
	// Scatter range: bounding box of the boundary polygon
	float h0 = 0.0f, h1 = 0.0f;
	for (const Pt2 &p : inBoundary)
	{
		h0 = JPH::max(h0, JPH::abs(p.x));
		h1 = JPH::max(h1, JPH::abs(p.y));
	}

	uint32 rng = inSeed;
	auto nextf = [&]() -> float {
		rng = rng * 1664525u + 1013904223u;
		return float(rng >> 8) / float(1u << 24);
	};

	Array<Pt2> seeds(inNumPieces);
	for (Pt2 &s : seeds)
		s = { nextf() * 2.0f * h0 - h0, nextf() * 2.0f * h1 - h1 };

	for (int i = 0; i < inNumPieces; ++i)
	{
		Array<Pt2> cell = inBoundary; // start from the actual shape boundary
		for (int j = 0; j < inNumPieces && !cell.empty(); ++j)
		{
			if (j == i) continue;
			float nx = seeds[j].x - seeds[i].x;
			float ny = seeds[j].y - seeds[i].y;
			float mx = (seeds[i].x + seeds[j].x) * 0.5f;
			float my = (seeds[i].y + seeds[j].y) * 0.5f;
			cell = sClipPolygon(cell, nx, ny, nx * mx + ny * my);
		}
		if ((int)cell.size() < 3) continue;

		float cx = 0.0f, cy = 0.0f;
		for (const Pt2 &p : cell) { cx += p.x; cy += p.y; }
		cx /= float(cell.size()); cy /= float(cell.size());

		Array<Vec3> verts;
		verts.reserve(cell.size() * 2);
		for (const Pt2 &p : cell)
		{
			verts.push_back(Vec3(p.x - cx, -inHalfThick, p.y - cy));
			verts.push_back(Vec3(p.x - cx,  inHalfThick, p.y - cy));
		}

		auto result = ConvexHullShapeSettings(verts.data(), (int)verts.size(), 0.0f).Create();
		if (!result.IsValid()) continue;

		outShapes.push_back(result.Get());
		outLocalCenters.push_back(Vec3(cx, 0.0f, cy));
	}
}

// ---------------------------------------------------------------------------
// Main demo wall layout constants (used by BuildMainWall only)
// ---------------------------------------------------------------------------

static constexpr int	cNumBays		= 3;
static constexpr int	cNumFloors		= 3;
static constexpr float	cBayWidth		= 2.0f;
static constexpr float	cFloorHeight	= 2.0f;
static constexpr float	cStubHeight		= 0.5f;

static const Vec3 cStubHalf		(0.1f,  cStubHeight * 0.5f,             0.1f);
static const Vec3 cColumnHalf	(0.1f,  cFloorHeight * 0.5f,            0.1f);
static const Vec3 cBeamHalf		(cBayWidth * 0.5f - 0.1f, 0.1f,        0.1f);
static const Vec3 cPanelHalf	(cBayWidth * 0.5f - 0.2f, cFloorHeight * 0.5f - 0.2f, 0.05f);

// ---------------------------------------------------------------------------
// BuildMainWall — the main demonstration wall (columns + beams + panels).
// Called from Initialize() after state is reset and the floor is created.
// ---------------------------------------------------------------------------

void DestructibleTest::BuildMainWall()
{
	RefConst<Shape> stub_shape	= new BoxShape(cStubHalf);
	RefConst<Shape> col_shape	= new BoxShape(cColumnHalf);
	RefConst<Shape> beam_shape	= new BoxShape(cBeamHalf);
	RefConst<Shape> panel_shape	= new BoxShape(cPanelHalf);

	Ref<GroupFilterTable> building_filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

	float col_x[cNumBays + 1];
	for (int j = 0; j <= cNumBays; ++j)
		col_x[j] = (j - cNumBays * 0.5f) * cBayWidth;

	auto addBody = [&](const BodyCreationSettings &inBCS, EActivation) -> Body *
	{
		Body *b = mBodyInterface->CreateBody(inBCS);
		b->SetCollisionGroup(CollisionGroup(building_filter, gid, 0));
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
			float y = is_base ? cStubHeight * 0.5f
			                  : cStubHeight + (i - 0.5f) * cFloorHeight;

			BodyCreationSettings bcs(is_base ? stub_shape : col_shape,
				RVec3(col_x[j], y, 0.0f), Quat::sIdentity(),
				is_base ? EMotionType::Static  : EMotionType::Dynamic,
				is_base ? Layers::NON_MOVING   : Layers::MOVING);

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
				EMotionType::Dynamic, Layers::MOVING);
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
				EMotionType::Dynamic, Layers::MOVING);
			bcs.mOverrideMassProperties			= EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass	= 20.0f;

			panel[j][i] = addBody(bcs, EActivation::Activate);

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
	// -----------------------------------------------------------------------
	for (int j = 0; j < cNumBays; ++j)
		for (int i = 1; i < cNumFloors; ++i)
		{
			TrackConstraint(false, col[j][i],     panel[j][i]);
			TrackConstraint(false, col[j + 1][i], panel[j][i]);
		}
}

// ---------------------------------------------------------------------------
// BuildHouse
// ---------------------------------------------------------------------------

void DestructibleTest::BuildHouse(RVec3Arg inCenter)
{
	static constexpr float hW    = 3.0f;
	static constexpr float hD    = 2.0f;
	static constexpr float hH    = 3.0f;
	static constexpr float hRise = 1.0f;

	const float slope_half = Sqrt(hD * hD + hRise * hRise) * 0.5f;
	const float roof_angle = atan2f(hRise, hD);

	static constexpr float hStub     = 0.5f;
	const float stub_half_h          = hStub * 0.5f;
	const float pillar_half_h        = (hH - hStub) * 0.5f;
	const float stub_cy              = stub_half_h;
	const float col_cy               = hStub + pillar_half_h;
	const float panel_half_h         = pillar_half_h - 0.1f;

	RefConst<Shape> s_stub  = new BoxShape(Vec3(0.1f, stub_half_h,  0.1f));
	RefConst<Shape> s_col   = new BoxShape(Vec3(0.1f, pillar_half_h, 0.1f));
	RefConst<Shape> s_bx    = new BoxShape(Vec3(hW * 0.5f - 0.1f,  0.1f,          0.1f));
	RefConst<Shape> s_bz    = new BoxShape(Vec3(0.1f,               0.1f, hD * 0.5f - 0.1f));
	RefConst<Shape> s_ridge = new BoxShape(Vec3(hW - 0.1f,          0.1f,          0.1f));
	RefConst<Shape> s_roof  = new BoxShape(Vec3(hW - 0.1f,         0.05f,      slope_half));
	RefConst<Shape> s_pwf   = new BoxShape(Vec3(hW * 0.5f - 0.2f, panel_half_h,          0.05f));
	RefConst<Shape> s_pws   = new BoxShape(Vec3(0.05f,             panel_half_h, hD * 0.5f - 0.15f));

	Ref<GroupFilterTable> house_filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

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
		b->SetCollisionGroup(CollisionGroup(house_filter, gid, 0));
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
	// -----------------------------------------------------------------------
	auto addColPair = [&](float x, float z) -> Body *
	{
		Body *stub   = addBody(RVec3(x, stub_cy, z), Quat::sIdentity(), s_stub,
			EMotionType::Static,  Layers::NON_MOVING, 0,     EActivation::DontActivate);
		Body *pillar = addBody(RVec3(x, col_cy,  z), Quat::sIdentity(), s_col,
			EMotionType::Dynamic, Layers::MOVING,  300.0f, EActivation::Activate);
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
	const float bcx = hW * 0.5f;
	const float bcz = hD * 0.5f;

	Body *bf1 = addBody(RVec3(-bcx, bcy, -hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *bf2 = addBody(RVec3(+bcx, bcy, -hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *bb1 = addBody(RVec3(-bcx, bcy, +hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *bb2 = addBody(RVec3(+bcx, bcy, +hD), Quat::sIdentity(), s_bx, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *bl1 = addBody(RVec3(-hW, bcy, -bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *bl2 = addBody(RVec3(-hW, bcy, +bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *br1 = addBody(RVec3(+hW, bcy, -bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);
	Body *br2 = addBody(RVec3(+hW, bcy, +bcz), Quat::sIdentity(), s_bz, EMotionType::Dynamic, Layers::MOVING, 50, EActivation::Activate);

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
	// Ridge beam
	// -----------------------------------------------------------------------
	Body *ridge = addBody(RVec3(0, hH + hRise, 0), Quat::sIdentity(), s_ridge, EMotionType::Dynamic, Layers::MOVING, 80, EActivation::Activate);
	registerHouseElement(ridge, 0, hH + hRise, 0, Vec3(hW - 0.1f, 0.1f, 0.1f), 3, true);

	// -----------------------------------------------------------------------
	// Roof panels (2 tilted slabs)
	// -----------------------------------------------------------------------
	Body *roof_f = addBody(RVec3(0, hH + hRise * 0.5f, -hD * 0.5f),
		Quat::sRotation(Vec3::sAxisX(), -roof_angle), s_roof,
		EMotionType::Dynamic, Layers::MOVING, 60, EActivation::Activate);
	registerHouseElement(roof_f, 0, hH + hRise * 0.5f, -hD * 0.5f,
		Vec3(hW - 0.1f, 0.05f, slope_half), cFracturePieces, true);

	Body *roof_b = addBody(RVec3(0, hH + hRise * 0.5f, +hD * 0.5f),
		Quat::sRotation(Vec3::sAxisX(), +roof_angle), s_roof,
		EMotionType::Dynamic, Layers::MOVING, 60, EActivation::Activate);
	registerHouseElement(roof_b, 0, hH + hRise * 0.5f, +hD * 0.5f,
		Vec3(hW - 0.1f, 0.05f, slope_half), cFracturePieces, true);

	// -----------------------------------------------------------------------
	// Wall infill panels (8 dynamic)
	// -----------------------------------------------------------------------
	const Vec3 pwf_he(hW * 0.5f - 0.2f, panel_half_h, 0.05f);
	const Vec3 pws_he(0.05f, panel_half_h, hD * 0.5f - 0.15f);

	Body *fp1 = addBody(RVec3(-bcx, col_cy, -hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *fp2 = addBody(RVec3(+bcx, col_cy, -hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *bp1 = addBody(RVec3(-bcx, col_cy, +hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *bp2 = addBody(RVec3(+bcx, col_cy, +hD), Quat::sIdentity(), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *lp1 = addBody(RVec3(-hW, col_cy, -bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *lp2 = addBody(RVec3(-hW, col_cy, +bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *rp1 = addBody(RVec3(+hW, col_cy, -bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);
	Body *rp2 = addBody(RVec3(+hW, col_cy, +bcz), Quat::sIdentity(), s_pws, EMotionType::Dynamic, Layers::MOVING, 20, EActivation::Activate);

	registerHousePanel(fp1, -bcx, col_cy, -hD, pwf_he);
	registerHousePanel(fp2, +bcx, col_cy, -hD, pwf_he);
	registerHousePanel(bp1, -bcx, col_cy, +hD, pwf_he);
	registerHousePanel(bp2, +bcx, col_cy, +hD, pwf_he);
	registerHousePanel(lp1, -hW,  col_cy, -bcz, pws_he);
	registerHousePanel(lp2, -hW,  col_cy, +bcz, pws_he);
	registerHousePanel(rp1, +hW,  col_cy, -bcz, pws_he);
	registerHousePanel(rp2, +hW,  col_cy, +bcz, pws_he);

	// -----------------------------------------------------------------------
	// Frame constraints
	// -----------------------------------------------------------------------
	frameC(col_FL, bf1);  frameC(col_FM, bf1);
	frameC(col_FM, bf2);  frameC(col_FR, bf2);
	frameC(col_BL, bb1);  frameC(col_BM, bb1);
	frameC(col_BM, bb2);  frameC(col_BR, bb2);
	frameC(col_FL, bl1);  frameC(col_ML, bl1);
	frameC(col_ML, bl2);  frameC(col_BL, bl2);
	frameC(col_FR, br1);  frameC(col_MR, br1);
	frameC(col_MR, br2);  frameC(col_BR, br2);

	frameC(bf1, ridge);  frameC(bf2, ridge);
	frameC(bb1, ridge);  frameC(bb2, ridge);

	frameC(bf1, roof_f);  frameC(bf2, roof_f);
	frameC(bb1, roof_b);  frameC(bb2, roof_b);
	panelC(ridge, roof_f);
	panelC(ridge, roof_b);

	// -----------------------------------------------------------------------
	// Panel constraints
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

// ---------------------------------------------------------------------------
// BuildApartment
// ---------------------------------------------------------------------------

void DestructibleTest::BuildApartment(RVec3Arg inCenter, int inNumFloors, float inHalfW, float inHalfD)
{
	static constexpr float aFloorH = 2.5f;
	static constexpr float aStubH  = 0.5f;
	const float stubHalf  = aStubH * 0.5f;
	const float colHalf   = aFloorH * 0.5f;
	const float panHalfH  = colHalf - 0.1f;

	RefConst<Shape> s_stub = new BoxShape(Vec3(0.1f, stubHalf, 0.1f));
	RefConst<Shape> s_col  = new BoxShape(Vec3(0.1f, colHalf,  0.1f));
	RefConst<Shape> s_bx   = new BoxShape(Vec3(inHalfW - 0.1f, 0.1f, 0.1f));
	RefConst<Shape> s_bz   = new BoxShape(Vec3(0.1f, 0.1f, inHalfD - 0.1f));
	RefConst<Shape> s_pwf  = new BoxShape(Vec3(inHalfW - 0.2f, panHalfH, 0.05f));
	RefConst<Shape> s_pws  = new BoxShape(Vec3(0.05f, panHalfH, inHalfD - 0.2f));
	RefConst<Shape> s_roof = new BoxShape(Vec3(inHalfW - 0.1f, 0.1f, inHalfD - 0.1f));
	RefConst<Shape> s_flr  = new BoxShape(Vec3(inHalfW - 0.1f, 0.08f, inHalfD - 0.1f));

	Ref<GroupFilterTable> apt_filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

	auto addBody = [&](RVec3Arg lPos, RefConst<Shape> sh, EMotionType mt, ObjectLayer ol, float mass) -> Body *
	{
		BodyCreationSettings bcs(sh, inCenter + lPos, Quat::sIdentity(), mt, ol);
		if (mt != EMotionType::Static)
		{
			bcs.mOverrideMassProperties          = EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass    = mass;
		}
		Body *b = mBodyInterface->CreateBody(bcs);
		b->SetCollisionGroup(CollisionGroup(apt_filter, gid, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};

	auto frameC = [&](Body *a, Body *b) { TrackConstraint(true,  a, b); };
	auto panelC = [&](Body *a, Body *b) { TrackConstraint(false, a, b); };

	auto regElem = [&](Body *b, RVec3Arg lPos, Vec3 he, int pieces, bool isFrame)
	{
		RVec3 wp = inCenter + lPos;
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = isFrame;
		sGenerateFractureShapes(he, seed, pieces, info.mShapes, info.mLocalCenters);
		mFractureData[b->GetID()] = info;
	};

	const float cx[2] = { -inHalfW, +inHalfW };
	const float cz[2] = { -inHalfD, +inHalfD };

	Body *stubs[2][2];
	Array<Body *> cols[2][2];

	for (int xi = 0; xi < 2; ++xi)
		for (int zi = 0; zi < 2; ++zi)
		{
			float x = cx[xi], z = cz[zi];
			stubs[xi][zi] = addBody(RVec3(x, stubHalf, z), s_stub, EMotionType::Static, Layers::NON_MOVING, 0.0f);
			for (int f = 0; f < inNumFloors; ++f)
			{
				float y = aStubH + colHalf + f * aFloorH;
				Body *seg = addBody(RVec3(x, y, z), s_col, EMotionType::Dynamic, Layers::MOVING, 200.0f);
				regElem(seg, RVec3(x, y, z), Vec3(0.1f, colHalf, 0.1f), 3, true);
				cols[xi][zi].push_back(seg);
			}
		}

	for (int xi = 0; xi < 2; ++xi)
		for (int zi = 0; zi < 2; ++zi)
		{
			frameC(stubs[xi][zi], cols[xi][zi][0]);
			for (int f = 0; f + 1 < inNumFloors; ++f)
				frameC(cols[xi][zi][f], cols[xi][zi][f + 1]);
		}

	for (int f = 0; f < inNumFloors; ++f)
	{
		float beam_y  = aStubH + (f + 1) * aFloorH;
		float panel_y = aStubH + colHalf + f * aFloorH;

		Body *cFL = cols[0][0][f];
		Body *cFR = cols[1][0][f];
		Body *cBL = cols[0][1][f];
		Body *cBR = cols[1][1][f];

		Body *bf = addBody(RVec3(0,        beam_y, -inHalfD), s_bx, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *bb = addBody(RVec3(0,        beam_y, +inHalfD), s_bx, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *bl = addBody(RVec3(-inHalfW, beam_y, 0),        s_bz, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *br = addBody(RVec3(+inHalfW, beam_y, 0),        s_bz, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		regElem(bf, RVec3(0,        beam_y, -inHalfD), Vec3(inHalfW - 0.1f, 0.1f, 0.1f),    3, true);
		regElem(bb, RVec3(0,        beam_y, +inHalfD), Vec3(inHalfW - 0.1f, 0.1f, 0.1f),    3, true);
		regElem(bl, RVec3(-inHalfW, beam_y, 0),        Vec3(0.1f, 0.1f, inHalfD - 0.1f),    3, true);
		regElem(br, RVec3(+inHalfW, beam_y, 0),        Vec3(0.1f, 0.1f, inHalfD - 0.1f),    3, true);

		frameC(cFL, bf); frameC(cFR, bf);
		frameC(cBL, bb); frameC(cBR, bb);
		frameC(cFL, bl); frameC(cBL, bl);
		frameC(cFR, br); frameC(cBR, br);

		Body *pf = addBody(RVec3(0,        panel_y, -inHalfD), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pb = addBody(RVec3(0,        panel_y, +inHalfD), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pl = addBody(RVec3(-inHalfW, panel_y, 0),        s_pws, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pr = addBody(RVec3(+inHalfW, panel_y, 0),        s_pws, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		regElem(pf, RVec3(0,        panel_y, -inHalfD), Vec3(inHalfW - 0.2f, panHalfH, 0.05f),   cFracturePieces, false);
		regElem(pb, RVec3(0,        panel_y, +inHalfD), Vec3(inHalfW - 0.2f, panHalfH, 0.05f),   cFracturePieces, false);
		regElem(pl, RVec3(-inHalfW, panel_y, 0),        Vec3(0.05f, panHalfH, inHalfD - 0.2f),   cFracturePieces, false);
		regElem(pr, RVec3(+inHalfW, panel_y, 0),        Vec3(0.05f, panHalfH, inHalfD - 0.2f),   cFracturePieces, false);

		panelC(cFL, pf); panelC(cFR, pf);
		panelC(cBL, pb); panelC(cBR, pb);
		panelC(cFL, pl); panelC(cBL, pl);
		panelC(cFR, pr); panelC(cBR, pr);

		Body *flr = addBody(RVec3(0, beam_y, 0), s_flr, EMotionType::Dynamic, Layers::MOVING, 200.0f);
		regElem(flr, RVec3(0, beam_y, 0), Vec3(inHalfW - 0.1f, 0.08f, inHalfD - 0.1f), cFracturePieces, false);
		mFractureData[flr->GetID()].mIsFloor = true;
		panelC(bf, flr); panelC(bb, flr);
		panelC(bl, flr); panelC(br, flr);

	}

	float roof_y = aStubH + inNumFloors * aFloorH;
	Body *roof = addBody(RVec3(0, roof_y, 0), s_roof, EMotionType::Dynamic, Layers::MOVING, 200.0f);
	regElem(roof, RVec3(0, roof_y, 0), Vec3(inHalfW - 0.1f, 0.1f, inHalfD - 0.1f), cFracturePieces, true);
	frameC(cols[0][0][inNumFloors - 1], roof);
	frameC(cols[1][0][inNumFloors - 1], roof);
	frameC(cols[0][1][inNumFloors - 1], roof);
	frameC(cols[1][1][inNumFloors - 1], roof);
}

// ---------------------------------------------------------------------------
// BuildHighrise
// ---------------------------------------------------------------------------

void DestructibleTest::BuildHighrise(RVec3Arg inCenter, int inNumFloors, int inFloorsPerSeg, float inHalfW, float inHalfD)
{
	static constexpr float hFloorH = 2.5f;
	static constexpr float hStubH  = 0.5f;

	const float stubHalf  = hStubH * 0.5f;
	const float segH      = float(inFloorsPerSeg) * hFloorH;
	const float segHalf   = segH * 0.5f;
	const float panHalfH  = hFloorH * 0.5f - 0.1f;
	const int   numSegs   = (inNumFloors + inFloorsPerSeg - 1) / inFloorsPerSeg;

	RefConst<Shape> s_stub = new BoxShape(Vec3(0.15f, stubHalf,          0.15f));
	RefConst<Shape> s_col  = new BoxShape(Vec3(0.15f, segHalf,           0.15f));
	RefConst<Shape> s_bx   = new BoxShape(Vec3(inHalfW - 0.1f,  0.1f,   0.1f));
	RefConst<Shape> s_bz   = new BoxShape(Vec3(0.1f,            0.1f,    inHalfD - 0.1f));
	RefConst<Shape> s_pwf  = new BoxShape(Vec3(inHalfW - 0.2f,  panHalfH, 0.05f));
	RefConst<Shape> s_pws  = new BoxShape(Vec3(0.05f,           panHalfH, inHalfD - 0.2f));
	RefConst<Shape> s_roof = new BoxShape(Vec3(inHalfW - 0.1f,  0.1f,    inHalfD - 0.1f));
	RefConst<Shape> s_flr  = new BoxShape(Vec3(inHalfW - 0.1f,  0.08f,   inHalfD - 0.1f));

	Ref<GroupFilterTable> filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

	auto addBody = [&](RVec3Arg lPos, RefConst<Shape> sh, EMotionType mt, ObjectLayer ol, float mass) -> Body *
	{
		BodyCreationSettings bcs(sh, inCenter + lPos, Quat::sIdentity(), mt, ol);
		if (mt != EMotionType::Static)
		{
			bcs.mOverrideMassProperties          = EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass    = mass;
		}
		Body *b = mBodyInterface->CreateBody(bcs);
		b->SetCollisionGroup(CollisionGroup(filter, gid, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};

	auto frameC = [&](Body *a, Body *b) { TrackConstraint(true,  a, b); };
	auto panelC = [&](Body *a, Body *b) { TrackConstraint(false, a, b); };

	auto regElem = [&](Body *b, RVec3Arg lPos, Vec3 he, int pieces, bool isFrame)
	{
		RVec3 wp = inCenter + lPos;
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = isFrame;
		sGenerateFractureShapes(he, seed, pieces, info.mShapes, info.mLocalCenters);
		mFractureData[b->GetID()] = info;
	};

	const float cx[2] = { -inHalfW, +inHalfW };
	const float cz[2] = { -inHalfD, +inHalfD };

	Body *stubs[2][2];
	Array<Body *> cols[2][2];

	for (int xi = 0; xi < 2; ++xi)
		for (int zi = 0; zi < 2; ++zi)
		{
			float x = cx[xi], z = cz[zi];
			stubs[xi][zi] = addBody(RVec3(x, stubHalf, z), s_stub, EMotionType::Static, Layers::NON_MOVING, 0.0f);
			for (int s = 0; s < numSegs; ++s)
			{
				float y = hStubH + segHalf + s * segH;
				Body *seg = addBody(RVec3(x, y, z), s_col, EMotionType::Dynamic, Layers::MOVING,
					200.0f * float(inFloorsPerSeg));
				regElem(seg, RVec3(x, y, z), Vec3(0.15f, segHalf, 0.15f), inFloorsPerSeg + 2, true);
				cols[xi][zi].push_back(seg);
			}
		}

	for (int xi = 0; xi < 2; ++xi)
		for (int zi = 0; zi < 2; ++zi)
		{
			frameC(stubs[xi][zi], cols[xi][zi][0]);
			for (int s = 0; s + 1 < numSegs; ++s)
				frameC(cols[xi][zi][s], cols[xi][zi][s + 1]);
		}

	for (int f = 0; f < inNumFloors; ++f)
	{
		int   s       = f / inFloorsPerSeg;
		float beam_y  = hStubH + float(f + 1) * hFloorH;
		float panel_y = hStubH + (float(f) + 0.5f) * hFloorH;

		Body *cFL = cols[0][0][s];
		Body *cFR = cols[1][0][s];
		Body *cBL = cols[0][1][s];
		Body *cBR = cols[1][1][s];

		Body *bf = addBody(RVec3(0,        beam_y, -inHalfD), s_bx, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *bb = addBody(RVec3(0,        beam_y, +inHalfD), s_bx, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *bl = addBody(RVec3(-inHalfW, beam_y, 0),        s_bz, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		Body *br = addBody(RVec3(+inHalfW, beam_y, 0),        s_bz, EMotionType::Dynamic, Layers::MOVING, 60.0f);
		regElem(bf, RVec3(0,        beam_y, -inHalfD), Vec3(inHalfW - 0.1f, 0.1f, 0.1f),    3, true);
		regElem(bb, RVec3(0,        beam_y, +inHalfD), Vec3(inHalfW - 0.1f, 0.1f, 0.1f),    3, true);
		regElem(bl, RVec3(-inHalfW, beam_y, 0),        Vec3(0.1f, 0.1f, inHalfD - 0.1f),    3, true);
		regElem(br, RVec3(+inHalfW, beam_y, 0),        Vec3(0.1f, 0.1f, inHalfD - 0.1f),    3, true);

		frameC(cFL, bf); frameC(cFR, bf);
		frameC(cBL, bb); frameC(cBR, bb);
		frameC(cFL, bl); frameC(cBL, bl);
		frameC(cFR, br); frameC(cBR, br);

		Body *pf = addBody(RVec3(0,        panel_y, -inHalfD), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pb = addBody(RVec3(0,        panel_y, +inHalfD), s_pwf, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pl = addBody(RVec3(-inHalfW, panel_y, 0),        s_pws, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		Body *pr = addBody(RVec3(+inHalfW, panel_y, 0),        s_pws, EMotionType::Dynamic, Layers::MOVING, 20.0f);
		regElem(pf, RVec3(0,        panel_y, -inHalfD), Vec3(inHalfW - 0.2f, panHalfH, 0.05f),   cFracturePieces, false);
		regElem(pb, RVec3(0,        panel_y, +inHalfD), Vec3(inHalfW - 0.2f, panHalfH, 0.05f),   cFracturePieces, false);
		regElem(pl, RVec3(-inHalfW, panel_y, 0),        Vec3(0.05f, panHalfH, inHalfD - 0.2f),   cFracturePieces, false);
		regElem(pr, RVec3(+inHalfW, panel_y, 0),        Vec3(0.05f, panHalfH, inHalfD - 0.2f),   cFracturePieces, false);

		panelC(cFL, pf); panelC(cFR, pf);
		panelC(cBL, pb); panelC(cBR, pb);
		panelC(cFL, pl); panelC(cBL, pl);
		panelC(cFR, pr); panelC(cBR, pr);

		Body *flr = addBody(RVec3(0, beam_y, 0), s_flr, EMotionType::Dynamic, Layers::MOVING, 300.0f);
		regElem(flr, RVec3(0, beam_y, 0), Vec3(inHalfW - 0.1f, 0.08f, inHalfD - 0.1f), cFracturePieces, false);
		mFractureData[flr->GetID()].mIsFloor = true;
		panelC(bf, flr); panelC(bb, flr);
		panelC(bl, flr); panelC(br, flr);
	}

	float  roof_y = hStubH + float(numSegs) * segH;
	Body  *roof   = addBody(RVec3(0, roof_y, 0), s_roof, EMotionType::Dynamic, Layers::MOVING, 400.0f);
	regElem(roof, RVec3(0, roof_y, 0), Vec3(inHalfW - 0.1f, 0.1f, inHalfD - 0.1f), cFracturePieces, true);
	for (int xi = 0; xi < 2; ++xi)
		for (int zi = 0; zi < 2; ++zi)
			frameC(cols[xi][zi][numSegs - 1], roof);
}

// ---------------------------------------------------------------------------
// BuildTower
// ---------------------------------------------------------------------------

void DestructibleTest::BuildTower(RVec3Arg inCenter, int inNumFloors, float inRadius, int inNumSides)
{
	static constexpr float tFloorH = 3.0f;
	static constexpr float tStubH  = 0.5f;
	static constexpr float cPi     = 3.14159265358979f;

	const float stubHalf  = tStubH * 0.5f;
	const float colHalf   = tFloorH * 0.5f;
	const float panHalfH  = colHalf - 0.15f;
	const float angleStep = 2.0f * cPi / float(inNumSides);
	const float chordHalf = inRadius * sinf(angleStep * 0.5f);

	RefConst<Shape> s_stub  = new BoxShape(Vec3(0.1f, stubHalf, 0.1f));
	RefConst<Shape> s_col   = new BoxShape(Vec3(0.1f, colHalf,  0.1f));
	RefConst<Shape> s_beam  = new BoxShape(Vec3(chordHalf - 0.12f, 0.1f,     0.1f));
	RefConst<Shape> s_panel = new BoxShape(Vec3(chordHalf - 0.20f, panHalfH, 0.05f));

	Array<Vec3> roof_verts;
	roof_verts.reserve(inNumSides * 2);
	for (int k = 0; k < inNumSides; ++k)
	{
		float a  = k * angleStep;
		float rx = (inRadius - 0.05f) * cosf(a);
		float rz = (inRadius - 0.05f) * sinf(a);
		roof_verts.push_back(Vec3(rx, -0.1f, rz));
		roof_verts.push_back(Vec3(rx,  0.1f, rz));
	}
	auto roof_result = ConvexHullShapeSettings(roof_verts.data(), (int)roof_verts.size(), 0.0f).Create();
	RefConst<Shape> s_roof = roof_result.IsValid()
		? roof_result.Get()
		: (RefConst<Shape>)new BoxShape(Vec3(inRadius * 0.85f, 0.1f, inRadius * 0.85f));

	// Floor slab shape — same octagonal cross-section, slightly thinner than roof
	Array<Vec3> floor_verts;
	floor_verts.reserve(inNumSides * 2);
	for (int k = 0; k < inNumSides; ++k)
	{
		float a  = k * angleStep;
		float rx = (inRadius - 0.1f) * cosf(a);
		float rz = (inRadius - 0.1f) * sinf(a);
		floor_verts.push_back(Vec3(rx, -0.08f, rz));
		floor_verts.push_back(Vec3(rx,  0.08f, rz));
	}
	auto floor_hull = ConvexHullShapeSettings(floor_verts.data(), (int)floor_verts.size(), 0.0f).Create();
	RefConst<Shape> s_floor = floor_hull.IsValid()
		? floor_hull.Get()
		: (RefConst<Shape>)new BoxShape(Vec3(inRadius * 0.85f, 0.08f, inRadius * 0.85f));

	// Precompute fracture boundary polygon (XZ vertices of the octagonal slab)
	Array<Pt2> floorPoly;
	floorPoly.reserve(inNumSides);
	for (int k = 0; k < inNumSides; ++k)
	{
		float a = k * angleStep;
		floorPoly.push_back({ (inRadius - 0.1f) * cosf(a), (inRadius - 0.1f) * sinf(a) });
	}

	Ref<GroupFilterTable> filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

	auto addBody = [&](RVec3Arg lPos, QuatArg rot, RefConst<Shape> sh, EMotionType mt, ObjectLayer ol, float mass) -> Body *
	{
		BodyCreationSettings bcs(sh, inCenter + lPos, rot, mt, ol);
		if (mt != EMotionType::Static)
		{
			bcs.mOverrideMassProperties          = EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass    = mass;
		}
		Body *b = mBodyInterface->CreateBody(bcs);
		b->SetCollisionGroup(CollisionGroup(filter, gid, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};

	auto frameC = [&](Body *a, Body *b) { TrackConstraint(true,  a, b); };
	auto panelC = [&](Body *a, Body *b) { TrackConstraint(false, a, b); };

	auto regElem = [&](Body *b, RVec3Arg lPos, Vec3 he, int pieces, bool isFrame)
	{
		RVec3 wp = inCenter + lPos;
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = isFrame;
		sGenerateFractureShapes(he, seed, pieces, info.mShapes, info.mLocalCenters);
		mFractureData[b->GetID()] = info;
	};

	Array<Body *> stubs(inNumSides);
	Array<Array<Body *>> cols(inNumSides);

	for (int k = 0; k < inNumSides; ++k)
	{
		float a  = k * angleStep;
		float cx = inRadius * cosf(a);
		float cz = inRadius * sinf(a);

		stubs[k] = addBody(RVec3(cx, stubHalf, cz), Quat::sIdentity(), s_stub,
			EMotionType::Static, Layers::NON_MOVING, 0.0f);

		cols[k].resize(inNumFloors);
		for (int f = 0; f < inNumFloors; ++f)
		{
			float y   = tStubH + colHalf + f * tFloorH;
			Body *seg = addBody(RVec3(cx, y, cz), Quat::sIdentity(), s_col,
				EMotionType::Dynamic, Layers::MOVING, 200.0f);
			regElem(seg, RVec3(cx, y, cz), Vec3(0.1f, colHalf, 0.1f), 3, true);
			cols[k][f] = seg;
		}
	}

	for (int k = 0; k < inNumSides; ++k)
	{
		frameC(stubs[k], cols[k][0]);
		for (int f = 0; f + 1 < inNumFloors; ++f)
			frameC(cols[k][f], cols[k][f + 1]);
	}

	for (int f = 0; f < inNumFloors; ++f)
	{
		float beam_y  = tStubH + (f + 1) * tFloorH;
		float panel_y = tStubH + colHalf + f * tFloorH;

		Array<Body *> ringBeams;
		ringBeams.reserve(inNumSides);
		for (int k = 0; k < inNumSides; ++k)
		{
			int   kn  = (k + 1) % inNumSides;
			float ak  = k  * angleStep;
			float akn = kn * angleStep;

			float ck_x = inRadius * cosf(ak),  ck_z = inRadius * sinf(ak);
			float cn_x = inRadius * cosf(akn), cn_z = inRadius * sinf(akn);
			float bx   = (ck_x + cn_x) * 0.5f;
			float bz   = (ck_z + cn_z) * 0.5f;

			Quat rot = Quat::sRotation(Vec3::sAxisY(), atan2f(ck_z - cn_z, cn_x - ck_x));

			Body *beam = addBody(RVec3(bx, beam_y, bz), rot, s_beam,
				EMotionType::Dynamic, Layers::MOVING, 50.0f);
			regElem(beam, RVec3(bx, beam_y, bz), Vec3(chordHalf - 0.12f, 0.1f, 0.1f), 3, true);
			frameC(cols[k][f], beam);
			frameC(cols[kn][f], beam);
			ringBeams.push_back(beam);

			Body *panel = addBody(RVec3(bx, panel_y, bz), rot, s_panel,
				EMotionType::Dynamic, Layers::MOVING, 20.0f);
			regElem(panel, RVec3(bx, panel_y, bz), Vec3(chordHalf - 0.20f, panHalfH, 0.05f), cFracturePieces, false);
			panelC(cols[k][f], panel);
			panelC(cols[kn][f], panel);
		}

		// Octagonal floor slab spanning the interior at beam level
		Body *flr = addBody(RVec3(0, beam_y, 0), Quat::sIdentity(), s_floor,
			EMotionType::Dynamic, Layers::MOVING, 300.0f);
		{
			RVec3 wp = inCenter + RVec3(0, beam_y, 0);
			uint32 fseed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
				^ int(float(wp.GetY()) * 100.0f) * 19349663u
				^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
			FractureInfo finfo;
			finfo.mIsFrame = false;
			finfo.mIsFloor = true;
			sGenerateFractureShapesPoly(floorPoly, 0.08f, fseed, cFracturePieces, finfo.mShapes, finfo.mLocalCenters);
			mFractureData[flr->GetID()] = finfo;
		}
		for (Body *b : ringBeams)
			panelC(b, flr);
	}

	float  roof_y = tStubH + inNumFloors * tFloorH;
	Body  *roof   = addBody(RVec3(0, roof_y, 0), Quat::sIdentity(), s_roof,
		EMotionType::Dynamic, Layers::MOVING, 500.0f);
	// Build the octagonal boundary for fracture clipping so debris matches the roof shape.
	{
		Array<Pt2> roofPoly;
		roofPoly.reserve(inNumSides);
		for (int k = 0; k < inNumSides; ++k)
		{
			float a = k * angleStep;
			roofPoly.push_back({ (inRadius - 0.05f) * cosf(a), (inRadius - 0.05f) * sinf(a) });
		}
		RVec3 wp = inCenter + RVec3(0, roof_y, 0);
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = true;
		sGenerateFractureShapesPoly(roofPoly, 0.1f, seed, cFracturePieces, info.mShapes, info.mLocalCenters);
		mFractureData[roof->GetID()] = info;
	}
	for (int k = 0; k < inNumSides; ++k)
		frameC(cols[k][inNumFloors - 1], roof);
}

// ---------------------------------------------------------------------------
// BuildEiffelTower
// ---------------------------------------------------------------------------

void DestructibleTest::BuildEiffelTower(RVec3Arg inCenter)
{
	constexpr int   NumLevels  = 12;
	constexpr float BaseWidth  = 120.0f; // full square side at ground
	constexpr float TopWidth   = 8.0f;   // full square side at top of taper
	constexpr float Height     = 300.0f; // height of the tapered body
	constexpr float BeamThick  = 2.0f;   // main leg cross-section
	constexpr float BraceThick = 1.2f;   // cross-brace / ring cross-section
	constexpr float cBase      = 0.5f;   // vertical offset so all bodies clear y=0

	// Corner signs and adjacent face pairs, matching the four sides of the square.
	constexpr float kSX[4] = { +1, +1, -1, -1 };
	constexpr float kSZ[4] = { +1, -1, +1, -1 };
	constexpr int kFaces[4][2] = { {0,1}, {2,3}, {0,2}, {1,3} };

	Ref<GroupFilterTable> filter = new GroupFilterTable(1);
	uint32 gid = mNextBuildingGroupID++;

	auto addBody = [&](RVec3Arg lPos, QuatArg rot, RefConst<Shape> sh,
	                   EMotionType mt, ObjectLayer ol, float mass) -> Body *
	{
		BodyCreationSettings bcs(sh, inCenter + lPos, rot, mt, ol);
		if (mt != EMotionType::Static)
		{
			bcs.mOverrideMassProperties       = EOverrideMassProperties::CalculateInertia;
			bcs.mMassPropertiesOverride.mMass = mass;
		}
		Body *b = mBodyInterface->CreateBody(bcs);
		b->SetCollisionGroup(CollisionGroup(filter, gid, 0));
		mBodyInterface->AddBody(b->GetID(), EActivation::DontActivate);
		return b;
	};

	auto frameC = [&](Body *a, Body *b) { TrackConstraint(true, a, b); };

	// Register a beam in mFractureData using its body-local half-extents.
	auto regBeam = [&](Body *b, Vec3 lCenter, Vec3 he)
	{
		RVec3 wp = inCenter + RVec3(lCenter);
		uint32 seed = uint32(int(float(wp.GetX()) * 100.0f) * 73856093u
			^ int(float(wp.GetY()) * 100.0f) * 19349663u
			^ int(float(wp.GetZ()) * 100.0f) * 83492791u);
		FractureInfo info;
		info.mIsFrame = true;
		sGenerateFractureShapes(he, seed, 3, info.mShapes, info.mLocalCenters);
		mFractureData[b->GetID()] = info;
	};

	// Create a dynamic BoxShape beam between two body-local points.
	auto addBeam = [&](Vec3 A, Vec3 B, float thick, float mass) -> Body *
	{
		Vec3 delta = B - A;
		float len = delta.Length();
		if (len < 1.0e-4f) return nullptr;
		Vec3 center = (A + B) * 0.5f;
		Quat rot = Quat::sFromTo(Vec3::sAxisY(), delta / len);
		Vec3 he(thick * 0.5f, len * 0.5f, thick * 0.5f);
		Body *b = addBody(RVec3(center), rot, new BoxShape(he),
		                  EMotionType::Dynamic, Layers::MOVING, mass);
		regBeam(b, center, he);
		return b;
	};

	// Static anchor stubs at the four base corners.
	const float h_base = BaseWidth * 0.5f;
	Body *stubs[4];
	for (int i = 0; i < 4; ++i)
	{
		Vec3 lPos(kSX[i] * h_base, cBase, kSZ[i] * h_base);
		stubs[i] = addBody(RVec3(lPos), Quat::sIdentity(),
		    new BoxShape(Vec3(0.5f, 0.5f, 0.5f)),
		    EMotionType::Static, Layers::NON_MOVING, 0.0f);
	}

	// legBeams[corner][level] — one angled body per corner per level.
	Body *legBeams[4][NumLevels];
	for (int i = 0; i < 4; ++i)
		for (int s = 0; s < NumLevels; ++s)
			legBeams[i][s] = nullptr;

	for (int level = 0; level < NumLevels; ++level)
	{
		float t0 = float(level)     / float(NumLevels);
		float t1 = float(level + 1) / float(NumLevels);
		// √t profile: width drops rapidly near base, slowly near top.
		float w0 = BaseWidth + (TopWidth - BaseWidth) * sqrtf(t0);
		float w1 = BaseWidth + (TopWidth - BaseWidth) * sqrtf(t1);
		float h0 = w0 * 0.5f, h1 = w1 * 0.5f;
		float y0 = cBase + t0 * Height;
		float y1 = cBase + t1 * Height;

		// Corner leg beams (4 angled structural columns per level)
		for (int i = 0; i < 4; ++i)
		{
			Vec3 A(kSX[i]*h0, y0, kSZ[i]*h0);
			Vec3 B(kSX[i]*h1, y1, kSZ[i]*h1);
			legBeams[i][level] = addBeam(A, B, BeamThick, 300.0f);
		}

		// Horizontal ring at the base of this level (4 side members)
		for (const auto &f : kFaces)
		{
			int a = f[0], b = f[1];
			Body *ring = addBeam(
			    Vec3(kSX[a]*h0, y0, kSZ[a]*h0),
			    Vec3(kSX[b]*h0, y0, kSZ[b]*h0),
			    BraceThick, 60.0f);
			frameC(legBeams[a][level], ring);
			frameC(legBeams[b][level], ring);
		}

		// X-braces: two crossing diagonals per face (4 faces × 2 = 8 per level)
		for (const auto &f : kFaces)
		{
			int a = f[0], b = f[1];
			Body *br1 = addBeam(
			    Vec3(kSX[a]*h0, y0, kSZ[a]*h0),
			    Vec3(kSX[b]*h1, y1, kSZ[b]*h1),
			    BraceThick, 60.0f);
			frameC(legBeams[a][level], br1);
			frameC(legBeams[b][level], br1);

			Body *br2 = addBeam(
			    Vec3(kSX[b]*h0, y0, kSZ[b]*h0),
			    Vec3(kSX[a]*h1, y1, kSZ[a]*h1),
			    BraceThick, 60.0f);
			frameC(legBeams[b][level], br2);
			frameC(legBeams[a][level], br2);
		}
	}

	// Vertical chains: stub → leg[0] → leg[1] → … → leg[NumLevels-1]
	for (int i = 0; i < 4; ++i)
	{
		frameC(stubs[i], legBeams[i][0]);
		for (int s = 0; s + 1 < NumLevels; ++s)
			frameC(legBeams[i][s], legBeams[i][s + 1]);
	}

	// Closing ring at the very top
	{
		float ht = TopWidth * 0.5f;
		float yt = cBase + Height;
		for (const auto &f : kFaces)
		{
			int a = f[0], b = f[1];
			Body *ring = addBeam(
			    Vec3(kSX[a]*ht, yt, kSZ[a]*ht),
			    Vec3(kSX[b]*ht, yt, kSZ[b]*ht),
			    BraceThick, 60.0f);
			frameC(legBeams[a][NumLevels - 1], ring);
			frameC(legBeams[b][NumLevels - 1], ring);
		}
	}

	// Antenna
	{
		float yt = cBase + Height;
		Body *ant = addBeam(Vec3(0, yt, 0), Vec3(0, yt + 35.0f, 0), BraceThick, 20.0f);
		for (int i = 0; i < 4; ++i)
			frameC(legBeams[i][NumLevels - 1], ant);
	}
}
