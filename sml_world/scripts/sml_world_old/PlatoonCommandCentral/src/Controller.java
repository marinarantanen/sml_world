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


import java.awt.event.*;

public class Controller implements ActionListener{

    /**
     * Controller for the platoon commander platCom.
     * 
     */ 
    public Model model = null;
    public MasterView view;
    
    public Controller(MasterView viewIn){
        view = viewIn;
    }

    public void actionPerformed(ActionEvent event){
        /*
         * Function that is triggered when user presses the execute command button
         */

        String commandId = view.getCommand();

        printToView(commandId);
        if (commandId.equals("1: echo state")) {
            printToView("Echoing state");
            echoState();
        } else {

        String targetString = view.getTarget();
        String paramString = view.getParam();


        model.sendCommand(commandId, targetString, paramString);
        }
    }

    public void printToView(String outString){
        /*
         * prints outString to the main view text area
         */

        view.printToView(outString);
    }

    public void connect(String ip, int portNoIn) {
        /*
         * Creates a model which holds the connection to the main simulation
         */

        if (model == null) {
            model = new Model(ip, portNoIn, this);
            printToView("Connected at port no: " + Integer.toString(portNoIn) + "\n");

            String[] commandIDs = model.getCommandIDs();
            updateCommandIDs(commandIDs);
            long[] vehLong = getVehicles();
            view.setTargets(vehLong);
            view.pack();
        } else {
            System.out.println("Already connected with someone");
        }

    }

    public void updateCommandIDs(String[] commandIDs){
        view.setCommandIDsList(commandIDs);
    }

    public void disconnect(){
        /*
         * Disconnects the user from the main simulation
         */
        
        model.closeConnections();
        model = null;
        System.out.println("Closed connections");
    }

    public long[] getVehicles(){
        /*
         * returns an array (long) that contains the list of all SmartVehicle ids
         */
        return model.getVehicles();
    }

    public void echoState(){
        /*
         * function that echos the state of the vehicles whose ids are returned from getTargetVehicles in view
         */

        String commandId = "1: echo state";
        String paramString = view.getEchoStateParamString();
        Object[] targetVehicles = view.getTargetVehicles();
        printToView(paramString);

        for (Object vehicleIDString: targetVehicles) {
            model.sendCommand(commandId, (String)vehicleIDString, paramString);
        }
    }

    public String[] getAttributes(){

        /*
         * gets the attributes from smlworld
         */
        return model.getAttributes();
    }
}
    
