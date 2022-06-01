/****
 * 
 * This file is a portion of the package UserInput, a library that provides 
 * an Arduino sketch with the ability to provide a simple command line UI over 
 * a Stream (e.g., Serial).
 * 
 * A command line is a sequence of ascii characters terminated by a '\r'. It
 * is subdivided into "words" separated by ' ' characters. The first word in
 * a command line is the name of the command. The rest are parameters.
 * 
 * The basic scheme here is to instantiate a global UserInput object, passing 
 * it the Stream it is to use for communication. Then, in the sketch's setup() 
 * function, to attach handler functions, one for each command. Later, during 
 * execution, the handlers will be invoked to process their corresponding 
 * commands. Finally, in the sketch's loop() function, to call the UserInput 
 * object's run() function repeatedly. 
 * 
 * The run function reads from the Stream as bytes become available, 
 * buffering them and returning, until a '\r' is received. When run () sees a 
 * '\r' it looks at the first white-space-delimited word to determine which 
 * command the user has entered, and invokes the corresponding handler, if one 
 * exists. If none exists, run() invokes the default command handler. When the 
 * handler returns, the received user input is forgotten about, and the 
 * process repeats.
 * 
 * Command handlers can use the UserInput object's getWord() member function 
 * to gain convenient access to the individual words that make up the command 
 * line, or the getCommandLine() member function to retrieve the whole command 
 * line. Because getWord() returns a String, handlers can use the String 
 * member functions for further processing. For example, 
 * 
 *     int16_t amount = ui.getWord(1).toInt();
 * 
 * would retrieve the first parameter as an integer.
 * 
 *****
 * 
 * UserInput V0.2, September 2020
 * Copyright (C) 2020 D.L. Ehnebuske
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE. 
 * 
 ****/

#ifndef USER_INPUT_LIB
    #define USER_INPUT_LIB

    #if ARDUINO >= 100
        #include "Arduino.h"
    #else
         #include "WProgram.h"
    #endif

//  #define DEBUG_USER_INPUT                // Uncomment to enable debug printing on Stream stream
    #define MAX_HANDLERS        (16)        // The maximum number of callbacks
    #define UIL_PROMPT          (F("> "))   // User prompt string

    extern "C" {
	    // User-supplied handler functions always follow the signature: void cmd(void);
	    typedef void(*userInputHandler) (void);
    }


    class UserInput {
        public:
            /**
             * 
             * UserInput: The constructor
             * 
             * Parms:
             *  Stream s        The Stream object to use to communicate with 
             *                  the user. Serial, for example.
             *  bool echo       Whether the characters received on s are to be 
             *                  echoed back on s for the user to see. 
             *                  Defaults to true.
             * 
             **/
            UserInput(Stream &s, bool echo = true);

            /**
             * 
             * attachDefaultCmdHandler():   Attach the default (unrecognized) 
             *                              cmd handler
             * 
             * Parms:
             *  userInputHandler handler    The user-supplied void function 
             *                              with no parameters to be called 
             *                              when an unrecognized cmd is 
             *                              encountered.
             * 
             **/
            void attachDefaultCmdHandler(userInputHandler handler);

            /**
             * 
             * attachCmdHandler():          Attach the cmd handler for a 
             *                              specified cmd.
             *
             * Parms:
             *  String cmd                  When the user inputs this string
             *                              invoke the corresponding handler
             *  userInputHandler handler    The user-supplied void function
             *                              with no parameters to be called
             *                              when use inputs cmd.
             * Returns:                     true if is succeeded, false if 
             *                              there are too many command 
             *                              handlers. See MAX_HANDLERS.
             * 
             **/
            bool attachCmdHandler(String cmd, userInputHandler handler);

            /**
             * 
             * run():                       Service the stream this object is 
             *                              responsible for dealing with. 
             *                              Invoke this member function
             *                              repeatedly; typically, every pass 
             *                              through loop().
             * 
             **/
            void run();

            /**
             * 
             * cancelCmd():                 Cancel any user input and start 
             *                              command over. If echoing, reissue 
             *                              the prompt. (Useful if some 
             *                              asynchronous action has occurred 
             *                              and the user should start any 
             *                              incomplete command from scratch.)
             * 
             **/
            void cancelCmd();

            /**
             * 
             * getWord():                   Return the specified "word" from 
             *                              the text the user has input as a 
             *                              String. A "word" is a white-space-
             *                              bounded sequence of characters. 
             *                              The words are numbered starting 
             *                              with 0. Word 0 is the cmd. If the 
             *                              user has not input the specified 
             *                              word, an empty String is returned.
             * Parms:
             *  int16_t ix                  The number of the word to be 
             *                              returned. Defaults to 0.
             * 
             **/
            String getWord(int16_t ix = 0);

            /**
             * 
             * getCommandLine():            Return the trimmed sequence of 
             *                              characters the user has input
             *                              as a String.
             * 
             **/
            String getCommandLine();

        private:
            userInputHandler handlers[MAX_HANDLERS];    // Pointers to the command handlers
            String cmds[MAX_HANDLERS];                  // The commands that go with the handlers
            userInputHandler defaultHandler;            // Pointer to the handler to use if the cmd not recognized
            Stream *stream;                             // The Stream we service
            bool echoing;                               // Whether we are to echo user input
            int8_t handlerCount;                        // The count of registered cmd handlers
            String commandLine;                         // Where user input is accumulated
            bool newCmd;                                // True after a command is processed and before additional input
            void process();                             // Process the accumulated command.
    };
#endif