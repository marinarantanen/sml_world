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


import java.lang.Thread;
import java.io.BufferedReader;
import java.io.IOException;

public class InThread extends Thread{
    /*
     * This class simply reads the incoming data on the socket and passes it forward to model.handleJson().
     */

    private BufferedReader in;
    private Model model;
    private boolean done;

    public InThread(BufferedReader readerIn, Model modelIn){
        in = readerIn;
        model = modelIn;
    }

    public void run(){
        while (!done){
            try {
                String json = in.readLine();
                if (json == null) {
                    break;
                }
                model.handleJson(json);
            } catch(IOException e){
                done = true;
            }
        }
        model.shutdown();
    }
}

