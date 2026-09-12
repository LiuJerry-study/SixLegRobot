/* 步态基类：**占位文件，没有任何可执行实现**。
 * 下面只有一段注释形式的单腿用法示例（不会被编译），保留作为 API 速查。
 */

/*单腿移动示例

SingleLeg leg(0);
leg.init(10.0);
leg.setArcLift(3.0);            // 弧线原语默认抬腿 3cm，可调

leg.startPrimitive(PRIM_ARC_RETRACT, 600);   // 摆动相：抬腿收回
while (leg.isMoving()) leg.update();

leg.startPrimitive(PRIM_LINE_EXTEND, 800);   // 支撑相：贴地伸出推地
while (leg.isMoving()) leg.update();

*/


