#ifndef  IOUTILITIES
#define IOUTILITIES

#include "main.h"

namespace Core
{
	class IOUtilities
	{
	public:

		importer readTextFile(string file)
		{
			importer ptsReader = *new importer(file, 200, 1.0);
			ptsReader.readPts_p5();
			return ptsReader;
		}
	};
}

#endif // ! IOUTILITIES