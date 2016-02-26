import javax.swing.*;


public class CatView extends JFrame implements MasterView {
    /**
     *
     */
    private static final long serialVersionUID = 1L;
    JButton catButton;
    Controller controller;
    JPanel thePanel;

    public CatView()
    {
        thePanel = new JPanel();
        controller = new Controller(this);
        catButton = new JButton("do cat");
        catButton.addActionListener(controller);

        thePanel.add(catButton);

        add(thePanel);

        controller.connect("127.0.0.1", 34978);
    }

    @Override
    public void printToView(String outString) {
        // TODO Auto-generated method stub

    }

    @Override
    public void showEchoStateOptions() {
        // TODO Auto-generated method stub

    }

    @Override
    public String getEchoStateParamString() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public Object[] getTargetVehicles() {
        // TODO Auto-generated method stub
        return null;
    }

    @Override
    public void setCommandIDsList(String[] commandIDArray) {
        // TODO Auto-generated method stub

    }

    @Override
    public String getCommand() {
        // TODO Auto-generated method stub
        return "cat";
    }

    @Override
    public String getTarget() {
        // TODO Auto-generated method stub
        return "";
    }

    @Override
    public String getParam() {
        // TODO Auto-generated method stub
        return "";
    }

    @Override
    public void setTargets(long[] vehLong) {
        // TODO Auto-generated method stub

    }

    @Override
    public void pack() {
        // TODO Auto-generated method stub
        super.pack();
    }
    
}
