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

import java.util.*;
import java.awt.event.*;

public class SubPane extends JPanel implements ActionListener {
    /**
     * Sub pane is not a sub pane. It is the panel making up the window with the
     * echo state parameter selections.
     */
    private static final long serialVersionUID = 1L;
    Controller controller;
    JCheckBox[] checkboxes;
    JCheckBox[] checkboxesAttributes;
    JButton selectAllVehicles;
    JButton selectAllAttributes;
    GroupLayout layout;
    
    public SubPane(Controller controllerIn){
        controller = controllerIn;

        layout = new GroupLayout(this);
        layout.setAutoCreateGaps(true);
        layout.setAutoCreateContainerGaps(true);

        this.setLayout(layout);

        long[] vehicles = controller.getVehicles();

        String[] attributes = controller.getAttributes(); // {"des_vel", "fwd", "nfwd", "bwd", "nbwd"};

        checkboxes = new JCheckBox[vehicles.length];
        checkboxesAttributes = new JCheckBox[attributes.length];

        selectAllVehicles = new JButton("Select all");
        selectAllVehicles.addActionListener(this);

        selectAllAttributes = new JButton("Select all");
        selectAllAttributes.addActionListener(this);
        

        GroupLayout.ParallelGroup horizontalGroup = layout.createParallelGroup();
        GroupLayout.SequentialGroup verticalGroup = layout.createSequentialGroup();
        

        JLabel vehicleLabel = new JLabel("Vehicles:");
        JLabel attributeLabel = new JLabel("Attributes:");


        for (int i = 0; i < vehicles.length; i++) {
            checkboxes[i] = new JCheckBox(Long.toString(vehicles[i]));
        }

        for (int i = 0; i < attributes.length; i++) {
            checkboxesAttributes[i] = new JCheckBox(attributes[i]);

        }

        horizontalGroup.addComponent(vehicleLabel);
        GroupLayout.SequentialGroup vehicleHorizontalGroup = layout.createSequentialGroup();
        for (int i = 0; i < vehicles.length; i++){
            vehicleHorizontalGroup.addComponent(checkboxes[i]);
        }
        vehicleHorizontalGroup.addComponent(selectAllVehicles);

        horizontalGroup.addGroup(vehicleHorizontalGroup);

        horizontalGroup.addComponent(attributeLabel);

        GroupLayout.SequentialGroup attributeHorizontalGroup = layout.createSequentialGroup();
        for (int i = 0; i < attributes.length; i++){
            attributeHorizontalGroup.addComponent(checkboxesAttributes[i]);
        }
        attributeHorizontalGroup.addComponent(selectAllAttributes);

        horizontalGroup.addGroup(attributeHorizontalGroup);

        verticalGroup.addComponent(vehicleLabel);
        
        GroupLayout.ParallelGroup vehicleVerticalGroup = layout.createParallelGroup(GroupLayout.Alignment.BASELINE);
        for (int i = 0; i < vehicles.length; i++){
            vehicleVerticalGroup.addComponent(checkboxes[i]);
        }
        vehicleVerticalGroup.addComponent(selectAllVehicles);

        verticalGroup.addGroup(vehicleVerticalGroup);

        verticalGroup.addComponent(attributeLabel);

        GroupLayout.ParallelGroup attributeVerticalGroup = layout.createParallelGroup(GroupLayout.Alignment.BASELINE);
        for (int i = 0; i < attributes.length; i++){
            attributeVerticalGroup.addComponent(checkboxesAttributes[i]);
        }
        attributeVerticalGroup.addComponent(selectAllAttributes);

        verticalGroup.addGroup(attributeVerticalGroup);

        layout.setHorizontalGroup(horizontalGroup);
        layout.setVerticalGroup(verticalGroup);

    }

    public String getParamString(){
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < checkboxesAttributes.length; i++) {
            if (checkboxesAttributes[i].isSelected()) {
                sb.append(checkboxesAttributes[i].getText() + ",");
            }
        }
    
        // just to remove ending comma
        if (sb.length() > 0) {
            sb.deleteCharAt(sb.length() - 1);
            return sb.toString();
        } else {
            return null;
        }

    }

    // All should be converted to strings before use
    public Object[] getSelectedVehicles(){
        List<String> selectedIDs = new ArrayList<String>();
        for ( int i = 0; i < checkboxes.length; i++) {
            if (checkboxes[i].isSelected()) {
                System.out.println(checkboxes[i].getText());
                selectedIDs.add(checkboxes[i].getText());
            }
        }
        return selectedIDs.toArray();
    }

    public void actionPerformed(ActionEvent event){
        if (event.getSource() == selectAllVehicles) {
            if (selectAllVehicles.getText().equals("Select all")) {
                for (JCheckBox checkbox : checkboxes) {
                    checkbox.setSelected(true);
                }
                selectAllVehicles.setText("Deselect all");
            } else {
                for (JCheckBox checkbox : checkboxes) {
                    checkbox.setSelected(false);
                }
                selectAllVehicles.setText("Select all");
            }
        } else if (event.getSource() == selectAllAttributes) {
            if (selectAllAttributes.getText().equals("Select all")) {
                for (JCheckBox checkbox : checkboxesAttributes) {
                    checkbox.setSelected(true);
                }
                selectAllAttributes.setText("Deselect all");
            } else {
                for (JCheckBox checkbox : checkboxesAttributes) {
                    checkbox.setSelected(false);
                }
                selectAllAttributes.setText("Select all");
            }
        }
    }

}
