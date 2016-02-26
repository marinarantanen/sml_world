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
import javax.swing.text.*;

public class View extends JFrame implements MasterView {

    /**
     * This is the normal view of platCom.
     * It features a big text window with capabilities of opening a echo state
     * frame. It also has text areas and a list of commands for commanding the
     * vehicles of SMLworld.
     */
    private static final long serialVersionUID = 1L;
    public Controller controller;
    public JPanel contentPane;
    public JPanel subPane;
    public String[] commandIDs =  {"echo-state","merge-after", "merge-between", "merge-before", "set-speed"}; 
    private JLabel targetLabel;
    private JLabel paramLabel;

    JMenuBar menuBar;

    // Main text area
    public JTextArea incomingText;

    private JScrollPane scrollPane;
    public JTextField targetField;
    public JTextField paramField;
    public JComboBox<String> list;
    public JButton execButton;
    public Model model;

    // View for the echoState checkboxes
    public JFrame echoStateOptionsView;

    // Pane that makes up the echostate checkboxes
    public SubPane optionsPane;
    
    public View(){
        /*
         * This constructor sets up the main view 
         */

        controller = new Controller(this);
        contentPane = new JPanel();
        GroupLayout layout = new GroupLayout(contentPane);
        contentPane.setLayout(layout);

        menuBar = new PlatoonMenuBar(controller, this);
        setJMenuBar(menuBar);

        layout.setAutoCreateGaps(true);
        layout.setAutoCreateContainerGaps(true);

        
        incomingText = new JTextArea(20, 30);
        incomingText.setEditable(false);

        DefaultCaret caret = (DefaultCaret)incomingText.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);

        scrollPane = new JScrollPane(incomingText);
        

        list = new JComboBox<String>();

        targetLabel = new JLabel("Target:");
        targetField = new JTextField(10);

        paramLabel = new JLabel("Parameters:");
        paramField = new JTextField(10);

        execButton = new JButton("Execute command");
        execButton.addActionListener(controller);

        layout.setHorizontalGroup(
                layout.createParallelGroup()
                .addComponent(scrollPane)
                .addGroup(layout.createSequentialGroup()
                    .addComponent(list)
                    .addComponent(targetLabel)
                    .addComponent(targetField)
                    .addComponent(paramLabel)
                    .addComponent(paramField)
                    .addComponent(execButton)
                    )
                );

        layout.setVerticalGroup(
                layout.createSequentialGroup()
                .addComponent(scrollPane)
                .addGroup(layout.createParallelGroup(GroupLayout.Alignment.BASELINE)
                    .addComponent(list)
                    .addComponent(targetLabel)
                    .addComponent(targetField)
                    .addComponent(paramLabel)
                    .addComponent(paramField)
                    .addComponent(execButton))
                );

        /*

        contentPane.add(list);

        contentPane.add(targetLabel);
        contentPane.add(targetField);
        contentPane.add(paramLabel);
        contentPane.add(paramField);

        contentPane.add(execButton);

        */
        add(contentPane);
        pack();



        }

    public void printToView(String outString){

        /*
         * Prints String outString to the main text area
         */

        incomingText.append(outString);
        incomingText.append("\n");

        DefaultCaret caret = (DefaultCaret)incomingText.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);

    }

    public void showEchoStateOptions(){

        /*
         * Makes the echo-state checkboxes visible
         */

        echoStateOptionsView = new JFrame();

        optionsPane = new SubPane(controller);

        echoStateOptionsView.add(optionsPane);

        echoStateOptionsView.pack();
        echoStateOptionsView.setVisible(true);


    }

    public String getEchoStateParamString() {
        /*
         * gets the parameter string for the echo state command, now using checkboxes
         */

        return optionsPane.getParamString();
    }
    
    public Object[] getTargetVehicles(){
        /*
         * gets the targetVehicles for the echoState command
         */

        return optionsPane.getSelectedVehicles();
    }

    public void setCommandIDsList(String[] commandIDArray){
        commandIDs = commandIDArray;
        list.removeAllItems();
        for (String commandID : commandIDs) {
            list.addItem(commandID);
        }
    }

    public String getCommand(){
        return (String)list.getSelectedItem();
    }

    public String getTarget(){
        return targetField.getText();
    }

    public String getParam(){
        return paramField.getText();
    }

    public void setTargets(long[] vehLong){

    }

    public void pack(){
        super.pack();
    }
}
