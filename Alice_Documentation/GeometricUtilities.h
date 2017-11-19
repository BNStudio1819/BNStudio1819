#ifndef  GEOMETRICUTILITIES
#define GEOMETRICUTILITIES

#include "main.h"

namespace Core
{

	class GeometricUtilities
	{
	public:

		GeometricUtilities() {};

		void LineLineClosestPoints(vec u, vec v, vec w, double &tu, double &tv)
		{
			double uu = u * u;
			double vv = v * v;

			double uv = u * v;
			double uw = u * w;
			double vw = v * w;

			double denom = 1.0 / (uu * vv - uv * uv);
			tu = (uv * vw - vv * uw) * denom;
			tv = (uu * vw - uv * uw) * denom;
		}

		vec LineLineShortestVector(vec startA, vec endA, vec startB, vec endB)
		{
			vec u = endA - startA;
			vec v = endB - startB;
			vec w = startA - startB;
			double tu, tv;

			LineLineClosestPoints(u, v, w, tu, tv);

			return  v * tv - u * tu - w;
		}

		vec projectVector(vec startA, vec endA, vec startB, vec endB)
		{
			vec u = endA - startA;
			vec v = endB - startB;

			//if ^ between the two vectors is < 90 => same direction, else 90 < ^ > 180 => opposite direction
			vec projected = (v / v.mag()) * (u * v);
			return projected;
		}


		float angle2DVec(vec a, vec b)
		{
			float angleDeg = (acos(a *b) * 180) / PI;
			return angleDeg;
		}

	};
}

#endif // ! GEOMETRICUTILITIES
