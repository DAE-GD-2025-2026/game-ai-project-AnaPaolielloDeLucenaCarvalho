#pragma once

namespace GameAI::Utilities::Geo
{
	inline float DistanceSquarePointToLine(const FVector2D& p1, const FVector2D& p2, const FVector2D& point)
	{
		//http://totologic.blogspot.be/2014/01/accurate-point-in-triangle-test.html
		const float p1p2_squareDistance = FVector2D::DistSquared(p1, p2);
		const float dp = ((point.X - p1.X)*(p2.X - p1.X) + (point.Y - p1.Y)*(p2.Y - p1.Y)) / p1p2_squareDistance;
		if (dp < 0)
			return FVector2D::DistSquared(p1, point);
		if (dp <= 1)
		{
			const float pp1_squareDistance = FVector2D::DistSquared(point, p1);
			return pp1_squareDistance - dp * dp * p1p2_squareDistance;
		}
		return FVector2D::DistSquared(p2, point);
	}
	
	constexpr bool PointInTriangleBoundingBox(const FVector2D& p, const FVector2D& tip, const FVector2D& prev, const FVector2D& next)
	{
		const float xMin = std::min(tip.X, std::min(prev.X, next.X)) - FLT_EPSILON;
		const float xMax = std::max(tip.X, std::max(prev.X, next.X)) + FLT_EPSILON;
		const float yMin = std::min(tip.Y, std::min(prev.Y, next.Y)) - FLT_EPSILON;
		const float yMax = std::max(tip.Y, std::max(prev.Y, next.Y)) + FLT_EPSILON;
		return !(p.X < xMin || xMax < p.X || p.Y < yMin || yMax < p.Y);
	}
	
	inline FVector2D ProjectOnLineSegment(const FVector2D& segmentStart, const FVector2D& segmentEnd, const FVector2D& point, float offset = 0.0f)
	{
		//Shorten segment based on offset, before doing actual calculations
		const FVector2D toStartDirection = FVector2D{segmentStart - segmentEnd}.GetSafeNormal();
		const FVector2D vEnd = segmentEnd + (toStartDirection * offset);
		const FVector2D toEndDirection = FVector2D{segmentEnd - segmentStart}.GetSafeNormal();
		const FVector2D vStart = segmentStart + (toEndDirection * offset);
		//Create shorten segment
		const FVector2D line = vEnd - vStart;

		//Projection
		const FVector2D w = point - vStart;
		const float proj = w.Dot(line);
		const float offsetPercentage = offset / line.Length();

		//Not on line or on vertex, get closest point and offset
		if (proj <= 0)
			return vStart;
		else
		{
			const float vsq = line.Dot(line);
			//Again not on line or on vertex, but the other side
			if (proj >= vsq)
				return vEnd;
			else
				return vStart + (proj / vsq) * line;
		}
	}
	
	constexpr bool PointInTriangle(const FVector2D& point, const FVector2D& V0, const FVector2D& V1, const FVector2D& V2, bool onLineAllowed = false)
	{
		//Do bounding box test first
		if (!PointInTriangleBoundingBox(point, V0, V1, V2))
			return false;

		//Reference: http://www.blackpawn.com/texts/pointinpoly/default.html
		//Compute direction vectors
		const FVector2D v0 = V1 - V0;
		const FVector2D v1 = V2 - V0;
		const FVector2D v2 = point - V0;

		//Compute dot products
		const float dot00 = v0.Dot(v0);
		const float dot01 = v0.Dot(v1);
		const float dot02 = v0.Dot(v2);
		const float dot11 = v1.Dot(v1);
		const float dot12 = v1.Dot(v2);

		// Compute barycentric coordinates
		const float invDenom = 1 / (dot00 * dot11 - dot01 * dot01);
		const float u = (dot11 * dot02 - dot01 * dot12) * invDenom;
		const float v = (dot00 * dot12 - dot01 * dot02) * invDenom;

		// Check if point is not in triangle (== can be on edge/line)
		if (u < 0 || v < 0 || u > 1 || v > 1 || (u + v) > 1)
		{
			if (onLineAllowed)
			{
				//Check special case where these barycentric coordinates are not enough for on line detection!
				if (DistanceSquarePointToLine(V0, V2, point) <= FLT_EPSILON ||
					DistanceSquarePointToLine(V2, V1, point) <= FLT_EPSILON ||
					DistanceSquarePointToLine(V1, V0, point) <= FLT_EPSILON)
					return true;
			}
			return false;
		}
		return true;
	}
}
