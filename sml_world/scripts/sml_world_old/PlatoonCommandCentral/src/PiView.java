/*

Command program for the SML world platoon implementation.
 
Copyright (C) 2015 Magnus Arvidsson

Permission is hereby granted, free of charge, to any person obtaining
a copy of this software and associated documentation files (the "Software"),
to deal in the Software without restriction, including without limitation
the rights to use, copy, modify, merge, publish, distribute, sublicense,
and/or sell copies of the Software, and to permit persons to whom the
Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included
in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE
OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

*/


import javax.swing.*;

public class PiView extends JFrame implements MasterView{
    /**
     * More simplistic view that does not have all features of View.
     * Written for a small RPi screen and thus does not have a big text area for
     * example.
     */
    private static final long serialVersionUID = 1L;
    long[] vehicles = {-1,-2, -3,-4};
    String[] vehString = new String[vehicles.length];
    String[] commandIDs = {"echo-state","merge-after", "merge-between", "merge-before", "set-speed"};


    
    JPanel mainPanel; 
    JComboBox<String> vehicleList;
    JComboBox<String> commandList;
    JButton exec;
    
    Controller controller;

    JMenuBar menuBar;
    
    public PiView(){
        for (int i = 0; i< vehicles.length; i++) {
            vehString[i] = Long.toString(vehicles[i]);
        }

        controller = new Controller(this);

        menuBar = new PlatoonMenuBar(controller, this);
        setJMenuBar(menuBar);

        
        vehicleList = new JComboBox<String>(vehString);
        commandList = new JComboBox<String>(commandIDs);

        exec = new JButton("Exe");
        exec.addActionListener(controller);

        mainPanel = new JPanel();

        mainPanel.add(commandList);
        mainPanel.add(vehicleList);
        mainPanel.add(exec);

        add(mainPanel);

    }

    public void printToView(String printString){
        System.out.println(printString);
    }

    public void showEchoStateOptions(){
        // Empty function
    }

    public String getEchoStateParamString(){
        return "vel";
    }

    public Object[] getTargetVehicles(){
        return null;
    }

    public void setCommandIDsList(String[] commandIDArray) {
        commandIDs = commandIDArray;
        commandList.removeAllItems();

        for (String commandID : commandIDs){
            commandList.addItem(commandID);
        }
    }

    public String getCommand(){
        return (String)commandList.getSelectedItem();
    }

    public String getTarget(){
        return (String)vehicleList.getSelectedItem();
    }

    public String getParam(){
        return "";
    }

    public void setTargets(long[] vehiclesIn){
        vehicles = vehiclesIn;
        vehString = new String[vehicles.length];
        for (int i = 0; i < vehicles.length; i++) {
            vehString[i] = Long.toString(vehicles[i]);
        }
        vehicleList.removeAllItems();

        for (String vehicleID : vehString){
            vehicleList.addItem(vehicleID);
        }
    }

    public void pack(){
        super.pack();
    }
}
