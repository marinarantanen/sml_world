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



import java.net.*;
import java.io.*;
import org.json.simple.*;
import org.json.simple.parser.*;
import java.util.*;


public class Model {
    /*
     * this class contains the connection to the main simulation and also the most recent list of vehicles.
     * Also has methods for parsing json coming from the simulation.
     */

    public int portNo;
    public Socket socket;
    public PrintWriter out;
    public BufferedReader in;
    public InThread inThread;
    private Object lock = new Object();
    private Object attributesLock = new Object();
    private Object commandIDsLock = new Object();

    private long[] vehicles;
    private String[] attributes;
    private String[] commandIDs;

    public Controller controller;

    public JSONParser parser = new JSONParser();

    private ContainerFactory factory = new ContainerFactory(){
        /*
         * Class used by the json-simple parser
         */

        public List creatArrayContainer(){
            return new ArrayList();
        }

        public Map createObjectContainer(){
            return new LinkedHashMap();
        }
    };

    public Model(String ip, int portNoIn, Controller controllerIn){
        portNo = portNoIn;
        controller = controllerIn;
        try {
            socket = new Socket(ip ,portNo);
            out = new PrintWriter(socket.getOutputStream(), true);
            in = new BufferedReader(new InputStreamReader(socket.getInputStream()));
            inThread = new InThread(in, this);
            inThread.start();

        } catch(IOException e){
            e.printStackTrace();
            System.exit(0);
        }

    }


        

    public void sendCommand(String commandId, String target,String params){
        /*
         * sends command to the main simulation
         *
         * Every request, including echo state and updateVehicles are routed through here
         */

    	System.out.println(commandId);
        System.out.println(target);
        try {
        int targetInt = Integer.parseInt(target);
        } catch (NumberFormatException e) {
            String targetInt = target;
        }
        params = params.replaceAll("\\s", "");
        String[] paramArray = params.split(",");
        JSONObject json = new JSONObject();
        json.put("commandID", commandId);
        try {
            int targetInt = Integer.parseInt(target);
            json.put("target", targetInt);
        } catch (NumberFormatException e) {
            json.put("target", target);
        }
        json.put("params", paramArray);

        out.println(json.toString());
    }

    public void print(String outString){
        /*
         * prints to main view
         */

        controller.printToView(outString);
    }


    public int handleJson(String jsonString){
        /*
         * function that handles the json strings from main simulation.
         * Depending on the comID in the jsonString it does different things.
         */

        print(jsonString);
        try {
            Map json = (Map)parser.parse(jsonString, factory);
            StringBuilder sb = new StringBuilder();

            String comID = (String)json.get("comID");

            // If updating vehicles list
            if (comID.equals("updateVehicles")) {
                updateVehicles(json);
                return 1;
            } else if (comID.equals("updateAttributes")) {
                updateAttributes(json);
                return 2;
            } else if (comID.equals("updateCommandIDs")) {
                updateCommandIDs(json);
                return 3;
            }

            // else
            

            sb.append("comID: " + comID + "; " );
            String vehicleID = Long.toString((long)(json.get("vehicleID")));
            sb.append("vehicleID: " + vehicleID + "\n");
            List fields = (List)json.get("fields");
            List values = (List)json.get("values");

            Iterator iterFields = fields.iterator();
            Iterator iterValues = values.iterator();

            while (iterFields.hasNext() && iterValues.hasNext()) {
                sb.append((String)iterFields.next() + ": ");
                sb.append((String)iterValues.next() + "\n");
            }

            print(sb.toString());
            return 0;

        } catch (ParseException e){
            e.printStackTrace();
            return -1;
        }

    }

    public void shutdown(){
        /*
         * Shuts down the connection
         */

        try {
            in.close();
            out.close();
            socket.close();
        } catch(Exception e){
            e.printStackTrace();
        }

        print("Connection shutdown");

    }

    public void closeConnections(){
        /*
         * closes connections
         */

        try {
            socket.close();
            in.close();
            out.close();
        } catch (Exception e) {
            e.printStackTrace();
        }

    }

    public long[] getVehicles(){
        /* 
         * sends the request for getting the new vehicle list
         * 
         * then waits for response and returns list
         */

        try {
            synchronized (lock) {
                out.println("{\"commandID\":\"updateVehicles\", \"target\": null, \"params\":[]}");
                System.out.println("Waiting for vehicles");

                // wait for update vehicles
                lock.wait();
                System.out.println("vehicles recevived");
                Arrays.sort(vehicles);
                return vehicles;
            }

        } catch (InterruptedException e){
            e.printStackTrace();
            return null;
        }



    }

    private void updateVehicles(Map json){
        /*
         * updates vehicles when response comes (see getVehices() for request sending
         */

        List newVehicles = (List)json.get("vehicles");
        int length = newVehicles.size();
        vehicles = new long[length];
        for (int i = 0; i < length; i++) {
            vehicles[i] = (long)newVehicles.get(i);
        }
        synchronized (lock) {
            // notifies getVehices()
            lock.notifyAll();
        }
    }

    public String[] getAttributes(){
        /*
         * sends requests for updated attributes
         */

        try {
            synchronized (attributesLock) {
                out.println("{\"commandID\":\"updateAttributes\", \"target\": null, \"params\":[]}");

                System.out.println("Waiting for attributes");

                attributesLock.wait();
                
                Arrays.sort(attributes);
                return attributes;
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
            return null;
        }
    
    }

    private void updateAttributes(Map json) {

        /**
         * Updates the list of attributes that can be used by echo state
         * command.
         */

        List newAttributes = (List)json.get("attributes");
        int length = newAttributes.size();
        attributes = new String[length];

        for (int i = 0; i < length; i++) {
            attributes[i] = (String)newAttributes.get(i);
        }
        synchronized (attributesLock) {
            // notifies getAttributes()
            attributesLock.notifyAll();
        }
    }

    public String[] getCommandIDs(){

        /**
         * Gets the list of available commands for the platCom program. does so
         * by sending a request of updated commands.
         */

        try {
            synchronized (commandIDsLock) {
                out.println("{\"commandID\":\"updateCommandIDs\", \"target\": null, \"params\":[]}");

                commandIDsLock.wait();

                Arrays.sort(commandIDs);
                return commandIDs;
                
            }
        } catch(Exception e){
            e.printStackTrace();
            return null;
        }
    }

    private void updateCommandIDs(Map json){

        /**
         * When list of command ids is returned from SMLworld this updates the
         * private variable commandIDs. This is then returned from
         * getCommandIDs.
         */

        List newCommandIDs = (List)json.get("commandIDs");
        int length = newCommandIDs.size();
        commandIDs = new String[length];

        for (int i = 0; i < length; i++) {
            commandIDs[i] = (String)newCommandIDs.get(i);
        }
        synchronized (commandIDsLock) {
            // notifies getAttributes()
            commandIDsLock.notifyAll();
        }
   }

}
