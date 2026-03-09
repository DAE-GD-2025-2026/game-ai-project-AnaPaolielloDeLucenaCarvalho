#pragma once

namespace GameAI::HeuristicFunctions
{
	// Common typedef
	typedef float(*Heuristic)(float, float);
	
	//Manhattan distance
	static float Manhattan(float x, float y)
	{
		return x + y;
	}

	//Euclidean distance
	static float Euclidean(float x, float y)
	{
		return sqrtf(x * x + y * y);
	}

	//Sqrt Euclidean distance
	static float SqEuclidean(float x, float y)
	{
		return x * x + y * y;
	}

	//Octile distance
	static float Octile(float x, float y)
	{
		float constexpr f = 0.414213562373095048801f; // == sqrt(2) - 1;
		return x < y ? f * x + y : f * y + x;
	}

	//Chebyshev distance
	static float Chebyshev(float x, float y)
	{
		return std::max(x, y);
	}
};
