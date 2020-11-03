using UnityEngine;

public class Game
{
    public const int GAME_SPEED = 30; // 每秒多少帧
    public const int SQUARE_SIZE = 8; // 1米8elmos
    public const int CIRCLE_DIVS = 32768 * 2; //一圈多少 2PI
    public static int frameNum = 1;
    public static int mapx = 100;
    public static int mapz = 200;
    public static int GetSquare(Vector3 pos)
    {
        int x = (int)pos.x / SQUARE_SIZE;
        int z = (int)pos.z / SQUARE_SIZE;
        return x + z * mapx;
    }
    public static void Update()
    {
        ++frameNum;
    }
}