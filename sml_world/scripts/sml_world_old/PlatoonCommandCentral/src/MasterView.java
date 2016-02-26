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


public interface MasterView {

    /**
     * An interface for the views of platCom. This is to make shure you specify
     * all functions so that you do not call a function that does not exist.
     * This should ensure easy view writing.
     */
    public void printToView(String outString);

    public void showEchoStateOptions();

    public String getEchoStateParamString();

    public Object[] getTargetVehicles();

    public void setCommandIDsList(String[] commandIDArray);

    public String getCommand();

    public String getTarget();

    public String getParam();

    public void setTargets(long[] vehLong);

    // This should just be an interface for the regular pack function of JFrame
    public void pack();

}
    

