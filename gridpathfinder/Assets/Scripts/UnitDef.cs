﻿using UnityEngine;

public class UnitDef
{
    public float speed = 0f; //每秒移动多少个elmos
    public float maxAcc = 0f;
    public float maxDec = 0f;
    public float turnRate = 0f; //转向速度
    public bool turnInPlace = true; //是否可以原地转向
    //对于具有turnInPlace = false的单位，它定义了将减慢到的最小速度（实际执行转弯的速度可以更高，具体取决于角度差和turnRate）。
    public float turnInPlaceSpeedLimit = 0f;
    //对于turnInPlace = true的单位，定义转弯时开始制动的最大角度（度）。
    public float turnInPlaceAngleLimit = 0f;
    public bool collidable = true; //如果为false，则unit的对象不能与之碰撞
    public float mass = 0f; //物体的质量[1-1e6f]
    public bool pushResistant = false; //该单元可以被其他单元推吗？启用它并不能消除所有推动，但有明显的改进。
    public bool isImmobileUnit = false; //不可移动的物体，比如建筑物
    public UnitDef()
    {
        this.speed = 3.85f * Game.GAME_SPEED;
        this.maxAcc = 1.5f;
        this.maxDec = 2.4f;
        this.turnRate = 3000f;
        this.turnInPlace = false;
        //this.turnInPlaceSpeedLimit = this.turnRate / Game.CIRCLE_DIVS;
        //this.turnInPlaceSpeedLimit *= (Mathf.PI * 2 * Game.SQUARE_SIZE);
        //this.turnInPlaceSpeedLimit /= Mathf.Max(this.speed / Game.GAME_SPEED, 1.0f);
        //this.turnInPlaceSpeedLimit = Mathf.Min(this.speed, this.turnInPlaceSpeedLimit);
        this.turnInPlaceSpeedLimit = 3.85f;
        this.turnInPlaceAngleLimit = 0f;
        this.mass = 76.7674408f;
        this.collidable = true;
        this.pushResistant = false;
        this.isImmobileUnit = false;
    }
}