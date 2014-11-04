package oldPomdpCluster;

import java.io.*;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.Random;
import java.util.Scanner;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 13-12-22
 * Time: 上午11:12
 * To change this template use File | Settings | File Templates.
 */
public class Model {
    int states;
    int actions;
    int observations;
    double[] startpoint;
    ArrayList<double[][]> transList = new ArrayList<double[][]>();
    ArrayList<double[][]> obserList = new ArrayList<double[][]>();
    ArrayList<double[]> reward = new ArrayList<double[]>();




    //for speed up oldPomdpCluster.test.

    double[][][] transArray;
    double[][][] obserArray;
    double[][] rewardArray;

    public void initInerArray(){
        transArray=new double[transList.size()][transList.get(0).length][transList.get(0)[0].length];
        obserArray=new double[obserList.size()][obserList.get(0).length][obserList.get(0)[0].length];
        rewardArray=new double[reward.size()][reward.get(0).length];
        for(int i=0;i<transList.size();i++){
            for(int k=0;k<transList.get(0).length;k++){
                for(int m=0;m<transList.get(0)[0].length;m++){
                    transArray[i][k][m]=transList.get(i)[k][m];
                }
            }
        }

        for(int i=0;i<obserList.size();i++){
            for(int k=0;k<obserList.get(0).length;k++){
                for(int m=0;m<obserList.get(0)[0].length;m++){
                    obserArray[i][k][m]=obserList.get(i)[k][m];
                }
            }
        }

        for(int i=0;i<reward.size();i++){
            for(int k=0;k<reward.get(0).length;k++){
                rewardArray[i][k]=reward.get(i)[k];
            }
        }







    }



    //speed up oldPomdpCluster.test end.


    public double[] nextPointForHallway(double[] fore, int action, int observation){
        double[] next = new double[states];
        for(int endState = 0;endState<states;endState++){
            double numerator=0;
            double denominator=0;
            //double bs=fore[endState];
            double Ozsa = obserList.get(action)[endState][observation];
            double sumBsT=0;
            for(int startState =0;startState<states;startState++){
                sumBsT+=fore[startState]*transList.get(action)[startState][endState];
            }
            numerator=Ozsa*sumBsT;


            for(int tempS=0;tempS<states;tempS++){
                for(int tempE=0;tempE<states;tempE++){
                    if(transList.get(action)[tempS][tempE]==0) continue;
                    denominator+=fore[tempS]*transList.get(action)[tempS][tempE]
                            *obserList.get(action)[tempE][observation];
                }
            }


            if(denominator==0){
                next[0]=Double.NaN;
                return next;
            }

            next[endState]=numerator/denominator;


        }
        return next;
    }



    //for speed up oldPomdpCluster.test.
    public static double[] nextPointForHallwayStatic(Model model,double[] fore, int action, int observation){
        double[] next = new double[model.states];
        for(int endState = 0;endState<model.states;endState++){
            double numerator=0;
            double denominator=0;
            double bs=fore[endState];
            double Ozsa = model.obserList.get(action)[endState][observation];
            double sumBsT=0;
            for(int startState =0;startState<model.states;startState++){
                sumBsT+=fore[startState]*model.transArray[action][startState][endState];
            }
            numerator=Ozsa*sumBsT;

            for(int tempS=0;tempS<model.states;tempS++){
                for(int tempE=0;tempE<model.states;tempE++){
                    denominator+=fore[tempS]*model.transArray[action][tempS][tempE]
                            *model.obserArray[action][tempE][observation];
                }
            }


            if(denominator==0){
                next[0]=Double.NaN;
                return next;
            }

            next[endState]=numerator/denominator;


        }
        return next;
    }

    public double[] nextPoint(double[] fore, int action, int observation) throws NextPointNotExistException {
        double[] next = new double[states];
        for (int i = 0; i < states; i++) {
            double numerator = 0;
            double denominator = 0;
            double bs = fore[i];
            double Oasz = obserList.get(action)[i][observation];
            double sumBsT = 0;
            for (int k = 0; k < states; k++) {
                sumBsT += fore[k] * transList.get(action)[k][i];
            }
            numerator = Oasz * sumBsT;

            for (int m = 0; m < states; m++) {
                for (int n = 0; n < states; n++) {
                    denominator += fore[m] * transList.get(action)[m][n] * obserList.get(action)[n][observation];
                }
            }
            if (denominator == 0) {
                throw new NextPointNotExistException();


            }
            next[i] = numerator / denominator;
        }
        return next;
    }

    public void initModelFromFileForHallWay(File model) {
        Scanner scanner;
        try {
            scanner = new Scanner(model);
            while (scanner.hasNext()) {
                String temp = scanner.nextLine();
                if (temp.matches("start:")) {
                    String[] sp = scanner.nextLine().split(" ");
                    startpoint = new double[sp.length];
                    for (int i = 0; i < startpoint.length; i++) {
                        startpoint[i] = Double.valueOf(sp[i].trim());
                    }
                } else if (temp.matches("states:.*")) {
                    states = Integer.valueOf(temp.split(" ")[1]);
                } else if (temp.matches("actions:.*")) {
                    actions = Integer.valueOf(temp.split(" ")[1]);
                } else if (temp.matches("observations:.*")) {
                    observations = Integer.valueOf(temp.split(" ")[1]);
                }
                if (states != 0 && actions != 0 && observations != 0) break;

            }

            for (int i = 0; i < actions; i++) {
                double[][] tempTrans = new double[states][states];
                transList.add(tempTrans);
                double[][] tempObser = new double[states][observations];
                obserList.add(tempObser);
            }
            for (int i = 0; i < actions; i++) {
                double[] tempReward = new double[states];
                reward.add(tempReward);
            }

            while (scanner.hasNext()) {
                String temp = scanner.nextLine();
                if (temp.matches("T:.*:.*:.*")) {
                    String[] sp = temp.split(":");
                    int tempAction = Integer.valueOf(sp[1].trim());
                    int startState = Integer.valueOf(sp[2].trim());
                    int endState = Integer.valueOf(sp[3].split(" ")[1]);
                    double value = Double.valueOf(sp[3].split(" ")[2]);
                    transList.get(tempAction)[startState][endState] = value;
                } else if (temp.matches("T:.*:.*")) {
                    String[] tempValueList = scanner.nextLine().split(" ");

                    int startState = Integer.valueOf(temp.split(":")[2].trim());
                    for (int action = 0; action < transList.size(); action++) {
                        for (int endState = 0; endState < states; endState++) {
                            transList.get(action)[startState][endState] = Double.valueOf(tempValueList[endState]);

                        }
                    }


                } else if (temp.matches("O:.*:.*")) {
                    int endState = Integer.valueOf(temp.split(":")[2].trim());
                    String[] oList = scanner.nextLine().split("\\s+");

                    for (double[][] ol : obserList) {
                        for (int ob = 0; ob < observations; ob++) {
                            ol[endState][ob] = Double.valueOf(oList[ob].trim());
                        }
                    }


                } else if (temp.matches("R:.*:.*:.*:.*")) {
                    String[] sp = temp.split(":");
                    int endState = Integer.valueOf(sp[3].trim());
                    double val = Double.valueOf(temp.split(" ")[temp.split(" ").length - 1]);
                    for (int action = 0; action < actions; action++) {
                        reward.get(action)[endState] = val;
                    }


                }

            }


        } catch (FileNotFoundException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }
    }

    public void initModelFromFile(File model) {
        Scanner scanner;
        try {
            scanner = new Scanner(model);
            while (scanner.hasNext()) {
                String temp = scanner.nextLine();
                if (temp.matches("states:.*")) {
                    states = Integer.valueOf(temp.split(" ")[1]);
                } else if (temp.matches("actions:.*")) {
                    actions = Integer.valueOf(temp.split(" ")[1]);
                } else if (temp.matches("observations:.*")) {
                    observations = Integer.valueOf(temp.split(" ")[1]);
                }
                if (states != 0 && actions != 0 && observations != 0) break;
            }

            for (int i = 0; i < actions; i++) {
                double[][] tempTrans = new double[states][states];
                transList.add(tempTrans);
                double[][] tempObser = new double[states][observations];
                obserList.add(tempObser);
            }
            for (int i = 0; i < actions; i++) {
                double[] tempReward = new double[states];
                reward.add(tempReward);
            }

            while (scanner.hasNext()) {
                String temp = scanner.nextLine();
                if (temp.matches("^T.*")) {
                    int tempIndex = Integer.valueOf(temp.split(":")[1].trim());
                    for (int i = 0; i < states; i++) {
                        temp = scanner.nextLine();
                        String[] tempSplit = temp.split(" ");
                        for (int k = 0; k < states; k++) {
                            transList.get(tempIndex)[i][k] = Double.valueOf(tempSplit[k]);
                        }
                    }
                } else if (temp.matches("^O.*")) {
                    int tempIndex = Integer.valueOf(temp.split(":")[1].trim());
                    for (int i = 0; i < states; i++) {
                        temp = scanner.nextLine();
                        String[] tempSplit = temp.split(" ");
                        for (int k = 0; k < observations; k++) {
                            obserList.get(tempIndex)[i][k] = Double.valueOf(tempSplit[k]);
                        }
                    }

                } else if (temp.matches("^R.*")) {
                    int tempIndex = Integer.valueOf(temp.split(":")[1].trim());
                    for (int i = 0; i < states; i++) {
                        String[] tempSplit = temp.split(":");

                        int state = Integer.valueOf(temp.split(":")[2].trim());
                        reward.get(tempIndex)[state] =
                                Double.valueOf(temp.split(" ")[temp.split(" ").length - 1]);


                    }


                }
            }

        } catch (FileNotFoundException e) {
            System.out.println("model file not found.\n exit.");
            System.exit(1);
        }


    }

    public static void printPoint(double[] point) {
        for (int i = 0; i < point.length; i++) {
            System.out.print(point[i]);
            if (i < point.length - 1) {
                System.out.print(" ");
            } else {
                System.out.print("\n");
            }
        }
    }

    public static void main(String[] args) {
        double[] firstPoint = new double[]{0.017865, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857,
                0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.017857, 0.0, 0.0, 0.0, 0.0};
        
        firstPoint=new double[]{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.5,0.0,0.0,0.0,0.0,0.0,0.0,0.0};
//        double[] firstPoint=new double[]{0.1,0.2,0.3,0.4} ;
        
        String modelFilePath = "tiger-grid.POMDP";//"e:\\hallway.POMDP";

        
        try {
//            writePointSetToFile(firstPoint, modelFilePath);
            writePointSetToFileSpreadFirst(firstPoint,modelFilePath);
        } catch (FileNotFoundException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }
    }

    public static void writePointSetToFileSpreadFirst(double[] point, String modelFilePath) throws FileNotFoundException {

        LinkedList<double[]> actionAndObvList = new LinkedList<double[]>();

        Model model = new Model();
//        model.initModelFromFile(new File(modelFilePath));
        model.initModelFromFileForHallWay(new File(modelFilePath));
        Random rand = new Random();
        if (new File("pointset").exists()) {
            new File("pointset").delete();

        }
        File writeFile = new File("pointset");

        PrintWriter pw = new PrintWriter(writeFile);

        double[] next = point;

        LinkedList<double[]> pointList = new LinkedList<double[]>();
        pointList.add(next);
        printPoint(next);

        for (int t = 0; t < next.length; t++) {
            pw.append(next[t] + "");
            if (t < next.length - 1) {
                pw.append(" ");
            } else {
                pw.append("\n");
            }
        }

        for (int i = 0; i < 3; i++) {
            LinkedList<double[] > tempNextPoints = new LinkedList<double[]>();

            for(int idx=0;idx<pointList.size();idx++){
                for(int act=0;act<model.actions;act++){
                    for(int obv=0;obv<model.observations;obv++){
                        double[] tempnext = model.nextPointForHallway(pointList.get(idx),act,obv);
                        tempNextPoints.add(tempnext);
                    }
                }
            }
            for(int m=0;m<tempNextPoints.size();m++){
                if(!new Double(tempNextPoints.get(m)[0]).isNaN()){
                    printPoint(tempNextPoints.get(m));
                    for (int t = 0; t < tempNextPoints.get(m).length; t++) {
                        pw.append(tempNextPoints.get(m)[t] + "");
                        if (t < tempNextPoints.get(m).length - 1) {
                            pw.append(" ");
                        } else {
                            pw.append("\n");
                        }
                    }
                }
            }

            pointList=tempNextPoints;



        }



        pw.close();


    }
    public static void writePointSetToFile(double[] point, String modelFilePath) throws FileNotFoundException {

        LinkedList<double[]> actionAndObvList = new LinkedList<double[]>();

        Model model = new Model();
//        model.initModelFromFile(new File(modelFilePath));
        model.initModelFromFileForHallWay(new File(modelFilePath));
        Random rand = new Random();
        if (new File("pointset").exists()) {
            new File("pointset").delete();

        }
        File writeFile = new File("pointset");

        PrintWriter pw = new PrintWriter(writeFile);

        double[] next = point;

        ArrayList<double[]> pointList = new ArrayList<double[]>();
        pointList.add(next);
        printPoint(next);

        for (int t = 0; t < next.length; t++) {
            pw.append(next[t] + "");
            if (t < next.length - 1) {
                pw.append(" ");
            } else {
                pw.append("\n");
            }
        }

        for (int i = 0; i < 50; i++) {

            double[] temp=new double[model.states];
            for(int k=0;k<next.length;k++){
                temp[k]=next[k];
            }



            while(true){
                int action = rand.nextInt(model.actions);
                int observation = rand.nextInt(model.observations);


                double[] actAndObv = new double[2];
                actAndObv[0]=action;
                actAndObv[1]=observation;
                actionAndObvList.add(actAndObv);




                next=model.nextPointForHallway(next,action,observation);

                if(new Double(next[0]).isNaN()){
                    for(int q=0;q<model.states;q++){
                        next[q]=temp[q];
                    }

                    actionAndObvList.remove(actionAndObvList.size()-1);
                }else{
                    break;
                }
            }

            printPoint(next);


            for (int t = 0; t < next.length; t++) {
                pw.append(next[t] + "");
                if (t < next.length - 1) {
                    pw.append(" ");
                } else {
                    pw.append("\n");
                }
            }

        }

        try {
            FileOutputStream fos = new FileOutputStream("actAndObv.out");
            ObjectOutputStream oos = new ObjectOutputStream(fos);
            oos.writeObject(actionAndObvList);
        } catch (FileNotFoundException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        } catch (IOException e) {
            e.printStackTrace();  //To change body of catch statement use File | Settings | File Templates.
        }


//        for (int i = 0; i < 5; i++) {
//            ArrayList<double[]> tempList = new ArrayList<double[]>();
//            for (int k = 0; k < pointList.size(); k++) {
//                printPoint(pointList.get(k));
//
//                for (int t = 0; t < pointList.get(k).length; t++) {
//                    pw.append(pointList.get(k)[t] + "");
//                    if (t < pointList.get(k).length - 1) {
//                        pw.append(" ");
//                    } else {
//                        pw.append("\n");
//                    }
//                }
//
//
//                for (int p = 0; p < model.actions; p++) {
//                    for (int q = 0; q < model.observations; q++) {
//                        if (model.obserList.get(p)[0][q] != 0)
//                            try {
//                                tempList.add(model.nextPoint(pointList.get(k), p, q));
//                            } catch (Exception e) {
//                                e.printStackTrace();
//
//                            }
//                    }
//                }
//            }
//            pointList = tempList;
//
//
//
//
//            int action = rand.nextInt(model.actions);
//            int observation = rand.nextInt(model.observations);
//            //~~~~
//            while (true) {
//                if (model.obserList.get(action)[0][observation] == 0) {
//                    observation = rand.nextInt(model.observations);
//                } else {
//                    break;
//                }
//            }
//            try {
//                next = model.nextPoint(next, action, observation);
//
//            } catch (Exception e) {
//                e.printStackTrace();
//            }
//
//
//
//        }

        pw.close();


    }





    public static void newWritePointSetToFile(double[] point, String modelFilePath) throws FileNotFoundException {
        Model model = new Model();
//        model.initModelFromFile(new File(modelFilePath));
        model.initModelFromFileForHallWay(new File(modelFilePath));
        Random rand = new Random();
        if (new File("pointset").exists()) {
            new File("pointset").delete();

        }
        File writeFile = new File("pointset");

        PrintWriter pw = new PrintWriter(writeFile);

        double[] next = point;



        LinkedList<double[]> bSet = new LinkedList<double[]>();
        LinkedList<double[]> bNextSet = new LinkedList<double[]>();



        if (bSet.size() == 0) {
            bSet.add(next);
            bNextSet = PomdpSolver.getAllNextPoints(bSet, model);
        }

        double[] newPoint = PomdpSolver.findMostDistPoint(bSet, bNextSet);
        bSet.add(newPoint);
        LinkedList<double[]> temp = new LinkedList<double[]>();
        temp.add(newPoint);
        bNextSet.addAll(PomdpSolver.getAllNextPoints(temp, model));

        double downcount=0;
        double len=0;
        while(bSet.size()<100){
            newPoint = PomdpSolver.findMostDistPoint(bSet, bNextSet);
            bSet.add(newPoint);
            temp = new LinkedList<double[]>();
            temp.add(newPoint);
            bNextSet.addAll(PomdpSolver.getAllNextPoints(temp, model));
            double tempLen =PomdpSolver.getLen(newPoint,bSet)/bSet.size();
//            if(tempLen>len){
//                len=tempLen;
//            }else{
//                downcount++;
//            }
            Model.printPoint(newPoint);

//            System.out.println(oldPomdpCluster.PomdpSolver.getLen(newPoint,bSet)/bSet.size());
//            System.out.println("bset size:"+bSet.size());

        }
        for(int i=0;i<bSet.size();i++){
            for (int t = 0; t < next.length; t++) {
                pw.append(bSet.get(i)[t] + "");
                if (t < bSet.get(i).length - 1) {
                    pw.append(" ");
                } else {
                    pw.append("\n");
                }
            }
        }
        pw.close();
        










//        ArrayList<double[]> pointList = new ArrayList<double[]>();
//        pointList.add(next);
//
//        for (int i = 0; i < 5000; i++) {
//            printPoint(next);
//            for (int t = 0; t < next.length; t++) {
//                pw.append(next[t] + "");
//                if (t < next.length - 1) {
//                    pw.append(" ");
//                } else {
//                    pw.append("\n");
//                }
//            }
//            double[] temp=new double[model.states];
//            for(int k=0;k<next.length;k++){
//                temp[k]=next[k];
//            }
//            while(true){
//                int action = rand.nextInt(model.actions);
//                int observation = rand.nextInt(model.observations);
//
//                next=model.nextPointForHallway(next,action,observation);
//                System.out.print("point:");
//                oldPomdpCluster.Model.printPoint(next);
//                if(new Double(next[0]).isNaN()){
//                    for(int q=0;q<model.states;q++){
//                        next[q]=temp[q];
//                    }
//                }else{
//                    break;
//                }
//            }
//        }
//        pw.close();
    }


}

class NextPointNotExistException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();    //To change body of overridden methods use File | Settings | File Templates.
        System.out.println("next point of these A O combination does not exist.");
    }
}


