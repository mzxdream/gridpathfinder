using UnityEngine;

public class Game
{
    public const int GAME_SPEED = 30; // 每秒多少帧
    public const int SQUARE_SIZE = 8; // 1米8elmos
    public const int CIRCLE_DIVS = 32768 * 2; //一圈多少 2PI
    public static int frameNum = 1;
    public static void Update()
    {
        ++frameNum;
    }
}