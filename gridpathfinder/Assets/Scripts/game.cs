using UnityEngine;

public class Game
{
    public const int GAME_SPEED = 30;
    public const int SQUARE_SIZE = 1;

    public static int frameNum = 1;
    public static void Update()
    {
        ++frameNum;
    }
}