#include "Polygon.h"
#include <GL/glu.h>

#include "InertiaTensor.h"

#include "PhysicEngine.h"

CPolygon::CPolygon(size_t index)
	: m_vertexBufferId(0), m_index(index), density(0.1f)
{
}

CPolygon::~CPolygon()
{
	DestroyBuffers();
}

void CPolygon::Build()
{
	m_lines.clear();

	ComputeArea();
	RecenterOnCenterOfMass();
	ComputeLocalInertiaTensor();

	CreateBuffers();
	BuildLines();
}

void CPolygon::Draw()
{
	// Set transforms (qssuming model view mode is set)
	float transfMat[16] = {	rotation.X.x, rotation.X.y, 0.0f, 0.0f,
							rotation.Y.x, rotation.Y.y, 0.0f, 0.0f,
							0.0f, 0.0f, 0.0f, 1.0f,
							position.x, position.y, -1.0f, 1.0f };
	glPushMatrix();
	glMultMatrixf(transfMat);

	// Draw vertices
	BindBuffers();
	glDrawArrays(GL_LINE_LOOP, 0, points.size());
	glDisableClientState(GL_VERTEX_ARRAY);

	glPopMatrix();
}

size_t	CPolygon::GetIndex() const
{
	return m_index;
}

float	CPolygon::GetArea() const
{
	return fabsf(m_signedArea);
}

Vec2	CPolygon::TransformPoint(const Vec2& point) const
{
	return position + rotation * point;
}

Vec2	CPolygon::InverseTransformPoint(const Vec2& point) const
{
	return rotation.GetInverseOrtho() * (point - position);
}

bool	CPolygon::IsPointInside(const Vec2& point) const
{
	float maxDist = -FLT_MAX;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		float pointDist = globalLine.GetPointDist(point);
		maxDist = Max(maxDist, pointDist);
	}

	return maxDist <= 0.0f;
}

bool	CPolygon::IsLineIntersectingPolygon(const Line& line, Vec2& colPoint, float& colDist) const
{
	//float dist = 0.0f;
	float minDist = FLT_MAX;
	Vec2 minPoint;
	float lastDist = 0.0f;
	bool intersecting = false;

	for (const Vec2& point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		float dist = line.GetPointDist(globalPoint);
		if (dist < minDist)
		{
			minPoint = globalPoint;
			minDist = dist;
		}

		intersecting = intersecting || (dist != 0.0f && lastDist * dist < 0.0f);
		lastDist = dist;
	}

	if (minDist <= 0.0f)
	{
		colDist = -minDist;
		colPoint = minPoint;
	}
	return (minDist <= 0.0f);
}

bool	CPolygon::CheckCollision(const CPolygon& poly, Vec2& colPoint, Vec2& colNormal, float& colDist) const
{
	float minDist = FLT_MAX;
	Vec2 minPoint, minNormal;
	bool separating = false;

	for (const Line& line : m_lines)
	{
		Line globalLine = line.Transform(rotation, position);
		Vec2 normal, point;
		float dist;
		separating = !poly.IsLineIntersectingPolygon(globalLine, point, dist) || separating;
		if (dist < minDist)
		{
			minDist = dist;
			minNormal = globalLine.GetNormal();
			minPoint = point;
		}
	}

	for (const Line& line : poly.m_lines)
	{
		Line globalLine = line.Transform(poly.rotation, poly.position);
		Vec2 normal, point;
		float dist;
		separating = !IsLineIntersectingPolygon(globalLine, point, dist) || separating;
		if (dist < minDist)
		{
			minDist = dist;
			minNormal = globalLine.GetNormal() * -1.0f;
			minPoint = point - minNormal * minDist;
		}
	}

	if (!separating)
	{
		colPoint = minPoint;
		colNormal = minNormal;
		colDist = minDist;
	}

	return !separating;
}

float CPolygon::GetInvSupport(const Vec2& point, const Vec2& dir, SFeature& feature)
{
	float maxDist = -FLT_MAX;

	for (size_t i = 0; i < m_lines.size(); ++i)
	{
		Vec2 normal = rotation * m_lines[i].GetNormal();
		bool bSegment = false;
		float dist = (TransformPoint(points[i]) - point) | dir;
		if (fabsf(normal | dir) >= 0.99f) //1.0f)
		{
			dist = Max(dist, (TransformPoint(points[(i + 1) % points.size()]) - point) | dir);
			bSegment = true;
		}
		
		if (dist > maxDist + Select(feature.bPoint, 0.0f, 0.0001f))
		{
			maxDist = dist;
			feature.bPoint = !bSegment;
			feature.index = i;
		}
	}

	return maxDist;
}


bool CPolygon::CheckCollision2(CPolygon& poly, SCollision& collision)
{
	float bestSupport = FLT_MAX;
	SFeature bestFeatureA;
	SFeature bestFeatureB;
	Vec2 bestNormal;
	Vec2 bestNormalDerivative;

	for (size_t i = 0; i < m_lines.size(); ++i)
	{
		Line globalLine = m_lines[i].Transform(rotation, position);

		SFeature feature;
		float support = poly.GetInvSupport(globalLine.point, globalLine.GetNormal() * -1.0f, feature);
		if (support < 0.0f)
		{
			// separation axis
			return false;
		}
		if (support < bestSupport)
		{
			bestSupport = support;
			bestFeatureA = SFeature(false, i);
			bestFeatureB = feature;
			bestNormal = globalLine.GetNormal();
			bestNormalDerivative = bestNormal.GetNormal() * angularVelocity;
		}
	}

	for (size_t i = 0; i < poly.m_lines.size(); ++i)
	{
		Line globalLine = poly.m_lines[i].Transform(poly.rotation, poly.position);

		SFeature feature;
		float support = GetInvSupport(globalLine.point, globalLine.GetNormal() * -1.0f, feature);
		if (support < 0.0f)
		{
			// separation axis
			return false;
		}
		if (support < bestSupport)
		{
			bestSupport = support;
			bestFeatureA = feature;
			bestFeatureB = SFeature(false, i);
			bestNormal = globalLine.GetNormal() * -1.0f;
			bestNormalDerivative = bestNormal.GetNormal() * poly.angularVelocity;
		}
	}

	collision.distance = bestSupport;
	collision.normal = bestNormal;
	collision.normalDerivative = bestNormalDerivative;
	
	return true;
}

float	CPolygon::GetSupport(const Vec2& center, const Vec2& dir) const
{
	float support = -FLT_MAX;
	for (const Vec2& point : points)
	{
		Vec2 globalPoint = TransformPoint(point);
		support = Max((globalPoint - center) | dir, support);
	}

	return support;
}


float	CPolygon::GetInvSupport(const Vec2& center, const Vec2& dir) const
{
	return -GetSupport(center, dir * -1.0f);

	//float support = FLT_MAX;
	//for (const Vec2& point : points)
	//{
	//	Vec2 globalPoint = TransformPoint(point);
	//	support = Min((globalPoint - center) | dir, support);
	//}

	//return support;
}

float	CPolygon::GetMaxSeparationEdge(size_t &edgeIndex, const CPolygon& poly) const
{
	float maxDist = -FLT_MAX;

	for (size_t i = 0; i < points.size(); ++i)
	{
		Line globalLine = m_lines[i].Transform(rotation, position);
		float dist = poly.GetInvSupport(globalLine.point, globalLine.GetNormal());
		edgeIndex = Select(dist > maxDist, i, edgeIndex);
		maxDist = Max(dist, maxDist);
	}

	return maxDist;
}

size_t	CPolygon::GetOpposingEdge(const Vec2& normal) const
{
	size_t edgeIndex = 0;
	float minDot = 1.0f;

	for (size_t i = 0; i < points.size(); ++i)
	{
		float dot = (rotation * m_lines[i].GetNormal()) | normal;
		edgeIndex = Select(dot < minDot, i, edgeIndex);
		minDot = Min(dot, minDot);
	}

	return edgeIndex;
}

bool	CPolygon::CheckCollision(CPolygon& poly, struct SCollision& collision)
{
	float threshold = 0;// 0.02f; // 0.01f;

	size_t aEdge = 0;
	float aSeparationDist = GetMaxSeparationEdge(aEdge, poly);
	if (aSeparationDist > 0.0f)
	{
		return false;
	}

	size_t bEdge = 0;
	float bSeparationDist = poly.GetMaxSeparationEdge(bEdge, *this);
	if (bSeparationDist > 0.0f)
	{
		return false;
	}

	collision.manifoldSize = 0;

	if (aSeparationDist > bSeparationDist + 0.1f)
	{
		Line aLine = m_lines[aEdge].Transform(rotation, position);

		Vec2 aNormal = aLine.GetNormal();
		size_t opposingEdge = poly.GetOpposingEdge(aNormal);
		Line opposingLine = poly.m_lines[opposingEdge].Transform(poly.rotation, poly.position);

		Vec2 points[2];
		opposingLine.GetPoints(*points, *(points + 1));
		Clip(aLine.point, aLine.dir, *points, *(points + 1));
		Clip(aLine.point + aLine.dir * aLine.length, aLine.dir * -1.0f, *points, *(points + 1));

		for (size_t i = 0; i < 2; ++i)
		{
			float dist = aLine.GetPointDist(points[i]);
			if (-dist >= -threshold)
			{
				collision.manifold[collision.manifoldSize].index = collision.manifoldSize;
				collision.manifold[collision.manifoldSize].normal = aNormal;
				collision.manifold[collision.manifoldSize].penetration = -dist;
				collision.manifold[collision.manifoldSize].point = points[i];
				collision.manifold[collision.manifoldSize].pA = this;
				collision.manifold[collision.manifoldSize].pB = &poly;

				collision.manifold[collision.manifoldSize].edgeNormalA = aNormal;
				collision.manifold[collision.manifoldSize].edgeNormalB = opposingLine.GetNormal();

				collision.manifoldSize++;
			}
		}

		
	}
	else
	{
		Line bLine = poly.m_lines[bEdge].Transform(poly.rotation, poly.position);

		Vec2 bNormal = bLine.GetNormal();
		size_t opposingEdge = GetOpposingEdge(bNormal);
		Line opposingLine = m_lines[opposingEdge].Transform(rotation, position);

		Vec2 points[2];
		opposingLine.GetPoints(*points, *(points + 1));

		Clip(bLine.point, bLine.dir, *points, *(points + 1));
		Clip(bLine.point + bLine.dir * bLine.length, bLine.dir * -1.0f, *points, *(points + 1));

		for (size_t i = 0; i < 2; ++i)
		{
			float dist = bLine.GetPointDist(points[i]);
			if (-dist >= -threshold)
			{
				collision.manifold[collision.manifoldSize].index = collision.manifoldSize;
				collision.manifold[collision.manifoldSize].normal = bNormal * -1.0f;
				collision.manifold[collision.manifoldSize].penetration = -dist;
				collision.manifold[collision.manifoldSize].point = points[i];
				collision.manifold[collision.manifoldSize].pA = this;
				collision.manifold[collision.manifoldSize].pB = &poly;

				collision.manifold[collision.manifoldSize].edgeNormalA = opposingLine.GetNormal();
				collision.manifold[collision.manifoldSize].edgeNormalB = bNormal;

				collision.manifoldSize++;
			}
		}
	}

	return true;
}

float CPolygon::UnProjectPoint(const Vec2& point, const Vec2& dir, float dist)
{
	float unProjectDist = 0.0f;

	for (size_t i = 0; (i < points.size()) && (unProjectDist < dist); ++i)
	{
		Line globalLine = m_lines[i].Transform(rotation, position);
		unProjectDist = Max(unProjectDist, globalLine.UnProject(point, dir));
	}

	return unProjectDist;
}

float CPolygon::UnProject(CPolygon& poly, const Vec2& dir, float dist)
{
	float unProjectDist = 0.0f;
	for (size_t i = 0; (i < points.size()) && (unProjectDist < dist); ++i)
	{
		Vec2 globalPoint = TransformPoint(points[i]);
		unProjectDist = Max(unProjectDist, poly.UnProjectPoint(globalPoint, dir, dist));
	}

	for (size_t i = 0; (i < poly.points.size()) && (unProjectDist < dist); ++i)
	{
		Vec2 globalPoint = poly.TransformPoint(poly.points[i]);
		unProjectDist = Max(unProjectDist, UnProjectPoint(globalPoint, dir * -1.0f, dist));
	}

	return (unProjectDist < dist)? unProjectDist : 0.0f;
}


void CPolygon::UpdateAABB()
{
	aabb.Center(position);
	for (const Vec2& point : points)
	{
		aabb.Extend(TransformPoint(point));
	}
}

float CPolygon::GetMass() const
{
	return density * GetArea();
}

float CPolygon::GetInertiaTensor() const
{
	return m_localInertiaTensor * GetMass();
}

Vec2 CPolygon::GetPointVelocity(const Vec2& point) const
{
	return speed + (point - position).GetNormal() * angularVelocity;
}

void CPolygon::CreateBuffers()
{
	DestroyBuffers();

	float* vertices = new float[3 * points.size()];
	for (size_t i = 0; i < points.size(); ++i)
	{
		vertices[3 * i] = points[i].x;
		vertices[3 * i + 1] = points[i].y;
		vertices[3 * i + 2] = 0.0f;
	}

	glGenBuffers(1, &m_vertexBufferId);

	glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);
	glBufferData(GL_ARRAY_BUFFER, sizeof(float) * 3 * points.size(), vertices, GL_STATIC_DRAW);

	glBindBuffer(GL_ARRAY_BUFFER, 0);

	delete[] vertices;
}

void CPolygon::BindBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glBindBuffer(GL_ARRAY_BUFFER, m_vertexBufferId);

		glEnableClientState(GL_VERTEX_ARRAY);
		glVertexPointer(3, GL_FLOAT, 0, (void*)0);
	}
}


void CPolygon::DestroyBuffers()
{
	if (m_vertexBufferId != 0)
	{
		glDeleteBuffers(1, &m_vertexBufferId);
		m_vertexBufferId = 0;
	}
}

void CPolygon::BuildLines()
{
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];

		Vec2 lineDir = (pointA - pointB).Normalized();

		m_lines.push_back(Line(pointB, lineDir, (pointA - pointB).GetLength()));
	}
}

void CPolygon::ComputeArea()
{
	m_signedArea = 0.0f;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		m_signedArea += pointA.x * pointB.y - pointB.x * pointA.y;
	}
	m_signedArea *= 0.5f;
}

void CPolygon::RecenterOnCenterOfMass()
{
	Vec2 centroid;
	for (size_t index = 0; index < points.size(); ++index)
	{
		const Vec2& pointA = points[index];
		const Vec2& pointB = points[(index + 1) % points.size()];
		float factor = pointA.x * pointB.y - pointB.x * pointA.y;
		centroid.x += (pointA.x + pointB.x) * factor;
		centroid.y += (pointA.y + pointB.y) * factor;
	}
	centroid /= 6.0f * m_signedArea;

	for (Vec2& point : points)
	{
		point -= centroid;
	}
	position += centroid;
}

void CPolygon::ComputeLocalInertiaTensor()
{
	m_localInertiaTensor = 0.0f;
	for (size_t i = 0; i + 1 < points.size(); ++i)
	{
		const Vec2& pointA = points[i];
		const Vec2& pointB = points[i + 1];

		m_localInertiaTensor += ComputeInertiaTensor_Triangle(Vec2(), pointA, pointB);
	}
}


