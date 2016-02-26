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


import java.awt.event.KeyEvent;
import java.awt.event.*;
import javax.swing.*;

public class PlatoonMenuBar extends JMenuBar implements ActionListener {
    /**
     * Menu bar of platCom.
     */
    private static final long serialVersionUID = 1L;
    private JMenu menu;
    private JMenuItem connect;
    private JMenuItem disconnect;
    private JMenuItem showEchoStateOptions;
    private Controller controller;
    private MasterView view;

    public PlatoonMenuBar(Controller controllerIn, MasterView viewIn){
        
        controller = controllerIn;
        view = viewIn;

        menu = new JMenu("Menu");
        menu.setMnemonic(KeyEvent.VK_M);
        add(menu);

        connect = new JMenuItem("Connect");
        connect.setAccelerator(KeyStroke.getKeyStroke(67, KeyEvent.ALT_MASK));
        menu.add(connect);
        connect.addActionListener(this);

        disconnect = new JMenuItem("Disconnect");
        disconnect.setAccelerator(KeyStroke.getKeyStroke(68, KeyEvent.ALT_MASK));
        menu.add(disconnect);
        disconnect.addActionListener(this);
        disconnect.setEnabled(false);

        showEchoStateOptions = new JMenuItem("Show echo_state options");
        showEchoStateOptions.setAccelerator(KeyStroke.getKeyStroke(70, KeyEvent.ALT_MASK));
        showEchoStateOptions.addActionListener(this);
        showEchoStateOptions.setEnabled(false);
        menu.add(showEchoStateOptions);
    }

    public void actionPerformed(ActionEvent event){
        if (event.getSource() == connect){
            try {
                String ip = (String)JOptionPane.showInputDialog("IP address:", "127.0.0.1");
                int portNo = Integer.parseInt((String)JOptionPane.showInputDialog("Port number:", 34978));
                connect.setEnabled(false);
                disconnect.setEnabled(true);
                controller.connect(ip, portNo);
                showEchoStateOptions.setEnabled(true);
            } catch (NumberFormatException e) {
                JOptionPane.showMessageDialog(new JFrame(), "Port number needs to be an integer");
            }
        } else if (event.getSource() == disconnect) {
            try {
                controller.disconnect();
                connect.setEnabled(true);
                disconnect.setEnabled(false);
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (event.getSource() == showEchoStateOptions) {
            try {
                view.showEchoStateOptions();
            } catch(Exception e){
                e.printStackTrace();
            }
        }
        
    }
}
