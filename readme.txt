PomdpSolver

main方法是执行算法的入口，包含的变量firstPoint为模型起始点，gama为折扣，modelFilePath为模型文件位置，暂时只能读取和hallway.POMDP格式一致的模型文件。Model.writePointSetToFile(firstPoint, modelFilePath)会把探索的可达点持久化到文件pointset。UtilClass.saveFSC(result,"XXXX.out")持久化求解的最优FSC到文件XXXX.out

UtilClass

main方法用于计算ADR。firstPoint为起始点。readFSC读取FSC文件。nTimesTryAve(10, 200, result, firstPoint, "XXXX.POMDP")表示XXXX模型模拟运行10次，每次200步。
