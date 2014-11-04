package newPomdpCluster.environments;
import java.io.*;
import java.util.ArrayList;
import java.util.Scanner;

/**
 * Created with IntelliJ IDEA.
 * User: hanbing
 * Date: 13-12-22
 * Time: 上午11:12
 * To change this template use File | Settings | File Templates.
 */
public class Model {
    public int states;
    int actions;
    public int observations;
    double[] startpoint;
    ArrayList<double[][]> transList = new ArrayList<double[][]>();
    ArrayList<double[][]> obserList = new ArrayList<double[][]>();
    public ArrayList<double[]> reward = new ArrayList<double[]>();




    //for speed up test.

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


   
 


class NextPointNotExistException extends Exception {
    @Override
    public void printStackTrace() {
        super.printStackTrace();    //To change body of overridden methods use File | Settings | File Templates.
        System.out.println("next point of these A O combination does not exist.");
    }
}
}


