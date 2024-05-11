#include <vector>
#include <stdlib.h>
#include <time.h>

using namespace std;

class SPoint2f
{
public:
    float x;
    float y;
};

class SONG_RANSAC
{
private:
    static bool HasSameValue(vector<int> vec);
    static vector<float> DirectLinearTransform(vector<SPoint2f> ref, vector<SPoint2f> dst);
    static bool IsInlierBySymmetricTransferError(SPoint2f ref, SPoint2f dst, float threshold, vector<float> H);
public:
    static vector<float> SRANSAC(vector<SPoint2f> ref, vector<SPoint2f> dst, float inlierRate, int maxIter, float threshold);
};