import processing.serial.*;

Serial myPort;
String val;            // serial read string from arduino
int LSR_flag = 0;      // LSR ON-OFF flag;  Sony 405nm 20mW laser
int LED_flag = 0;      // White LED ON-OFF flag; Samsung SPMWHT541MD5W ATMS3 LED
int[] data_all;        // data as converted from searial read
int[] data;            // clean spectra data
int[] data_diff;       // acquired spectra for subtraction
int[] data_stored;     // stored spectra for recovery if 'c' '[' or ']' entered
int draw_sum=0;        // flag for sum accumulation
int draw_diff=0;       // flag for difference plotting
double[] summed;       // accumulator container
int ignore_count=8;    // ignore n leading fields prior to spectrometer channel counts
                       // for plotting
int time_old=1;        // timestamp of prior sample in data[1], initialized to 1 (avoids undefined division)
int time_delta;        // difference between current and prior samples in data[1]
float samplerate;      // units are samples per sec; calculated as 
                       // 1000/(timestamp_new - timestamp_old);
float ADCres = pow(2,12);  // ADC resolution 12bit 4096, 10bit 1024, 16bit 65536
int draw_count=1;      // counter to skip 1st iterration of draw
float inttime_count =0;// keystroke counter: '[' and ']' integration time multiplier 
float intmult = 1;     // calculated integration time multiplier
PrintWriter output;

void setup() 
{
  println(Serial.list());
  String portName = Serial.list()[1]; //This is the index into the serial list
                                      //Starting index is 0
  println("Serial port path selected:  " + portName);
  println("Type 's' to toggle subtraction or direct measurement mode");
  println("Type 't' to toggle live or integration mode");
  println("Type 'c' to reset normalization");
  println("Type '[' or ']' to change integration time");
  println("Type (caps) 'L' to toggle Laser ON - OFF"); 
  println("Type (caps) 'W' to toggle LED ON - OFF");
  println("Type 'x' to save logging data and exit");  

  int[] dtstr = new int[6];  // define datestring variable to store in filename
    dtstr[0] = year();
    dtstr[1] = month();
    dtstr[2] = day();
    dtstr[3] = hour();
    dtstr[4] = minute();
    dtstr[5] = second();
  String jdtstr = join(nf(dtstr, 2), "");

  myPort = new Serial(this, portName, 115200);
    // Throw out the first reading, in case reading started in the middle of a string
    myPort.clear();
    val = myPort.readStringUntil('\n');
    val = null;
    // Crate file to store data locally;  modify path in future
  output = createWriter( jdtstr+"_hslr.dat" );
  
  summed = new double[288];
  for (int i = 0; i < 288; i++) 
  {
    summed[i] = 0;
  }                // initialize 'summed' var (double)
  size(626,550);   // size of display window: (288+25)x2 by 500+25x2
}

double find_max(double[] input)
{
  double max = 0;
  for (int i = 0; i < input.length; i++) 
  {
    if (input[i] > max)
    {
      max = input[i];
    }            
  }
  return max;
}

void plotdata()
{
  double summed_max = (find_max(summed)) / 500;   // normalization factor scaled
                                                  // to plot-window height (500)
  background(255);
        fill(0, 102, 153);     // blue
      rect(445, 30, 145, 20);  // rectangle for legend background
  if (draw_sum !=0)    // PLOTTING:  Normalized Spectrum
  {
        fill(255,0,0);
        textSize(12);
      text("Integration View", 450, 20);
        fill(0);
      text("Normalized Intensity",25,20);
      text("0", 10, 525);      // Intenisity y-axis labels
      text(".5", 10, 275);  
      text("1", 10, 25);
    for (int i=0; i < summed.length-1; i++)
    {
      line(25+i*2, 25+0, 25+i*2, 25+500 - (int)(summed[i]/summed_max));
    }
  }  
  else if (draw_diff !=0)   // PLOTTING:  Difference Spectrum
  {
        fill(255,0,0);
        textSize(12);
      text("Difference View", 450, 20);
        fill(0);
      text("Intensity Difference",25,20);
      text("-10", 5, 525);      // Intenisity y-axis labels
      text("0", 10, 275);  
      text("10", 5, 25);
    for (int i=0; i < summed.length-1; i++)
    {
      line(25+i*2, 25+0, 25+i*2, 25+250 - (int)(data[i] / 10));
//      line((float)(25+i*2), (float)(25+0), (float)(25+i*2), (float)(25+250) - (float)((data[i])/(find_max(temp))));
}
  }
  else                      // PLOTTING:  Live View
  {
        fill(255,0,0);
      text("Live View", 450, 20);
        fill(0);
      text("Intensity, DN (2^n-bits: 0 -> " + ADCres + ")",25,20);
      text("0", 10, 525);      // Intenisity y-axis labels
      text(".5", 10, 275);  
      text("1", 10, 25);
    for (int i=0; i < data.length-1; i++)
    {
      strokeWeight(3);
      line(25+i*2, 25+500 - (int)(data[i])*500/ADCres, 25+i*2, 25+0);
     }
} 
        fill(255);
      text("HSLR-v0 Spectrometer", 450, 45);
        noFill(); 
        stroke(0, 102, 153);
      rect(0, 0, 626, 550);   // Plot window outside border
        noFill(); 
        stroke(0, 102, 153);
      rect(25, 25, 576, 500);  // Plot window inside border
        strokeWeight(3);
        fill(0);
      text("Wavelength, nm", 500, 520);
      text("350", 50, 542);    // Wavelength x-axis labels
      text("450", 150, 542);
      text("550", 250, 542);
      text("650", 350, 542);
      text("750", 450, 542);
      text("850", 550, 542);
        fill(0, 102, 153);   // blue font
        textSize(16);
        // Display measured integration time in msec = micros() / 1000)
        // the nf() formats the float output leading zeros and decimal precision
      text("Int. time",210,505);
      text((nf((float)data_all[3]/1000,1,2)) + "msec", 280, 505);  
         textSize(12);
      text(intmult + " x int time mult", 210, 520);  // Display integration time multiplier    
        fill(0, 102, 153);   // blue font
        textSize(16);
        // Display measured sample period (msec) and rate (SPS)    
      text("Data Rate",40,505);
      text(nf((float)samplerate,1,1) + "Hz", 120, 505);// sample rate
        textSize(12);
      text(time_delta + "msec period", 40, 520);  // sample period
        // Display the spectrum counter above wavelength label
      text("spec# ",500,500);
      text(data_all[0], 535, 500);// Spectrum counter label
        textSize(12);
      
      if (LSR_flag == 1)  // Laser ON Warning
      {
          stroke(255, 255, 0);
          fill(255, 255, 0, 85);  // Yellow box with 85% transparency
        rect(370, 470, 115, 45);  // Plot rectangle around Warning
          stroke(0, 102, 153);
          fill(255, 0, 0);
        textSize(16);
        text("LASER ON !", 375, 490); // Laser ON Warning
        textSize(12);
      }
      if (LED_flag == 1)  // White LED ON Warning
      {
          stroke(255, 255, 0);
          fill(255, 255, 0, 85);
        rect(370, 470, 115, 45);  // Plot rectangle around Warning
          stroke(0, 102, 153);
          fill(255, 0, 0);
        textSize(16);
        text("White LED ON", 375, 510); // White LED ON Warning
        textSize(12);
      }
}

void draw()
{
  if ( myPort.available() > 0) 
  {  
    val = myPort.readStringUntil('\n');   // read it and store it in val

    if (val != null)
    {
        data_all = int(split(val, ','));  // split val string into int array
           // write only lines that contain data for all 288 channels
  
        if (data_all.length == (int)(288 + ignore_count))  
        {
        // Calculate time difference between current and previous millis() counter
        // in data_all[2] and store sample rate (in seconds) to print
        time_delta = data_all[2] - time_old;
        time_old = data_all[2];
        samplerate = 1000 / (float)(time_delta); 
          
        // Print data to file via serial port output.print
        output.print( val );
        data = subset(data_all, ignore_count);  // remove 1st n data from array
        data_stored = data;
        for (int i = 0; i < data.length; i++) 
        {
          if (draw_diff == 1)
          {
            data[i] = data[i]-data_diff[i];
          }          
          summed[i] +=  data[i];
        }

        plotdata();
        }
    }  
  }
}

void keyPressed() {
  if (key == 'c' ) 
  {
    //    println("Accumulator reset"); 
    for (int i = 0; i < summed.length; i++) 
    {
      summed[i] = 0;
    }
      if (draw_diff == 1)
        {
          draw_diff = 0; // turn-off difference plot
      }
  } 
  else if (key == 's' )  // toggle state of draw_diff
  {
    if (draw_diff == 1)
    {
        draw_diff = 0;  // turn-off difference plot
    }
    else
    {
        draw_diff = 1; // Turn on difference plot
        draw_sum = 0;  // Turn off draw_sum
        data_diff = data;
    }
  } 
  else if (key == 't' )  // toggle state of draw_sum
  {
    if (draw_sum == 1)
    {
      draw_sum = 0;  // turn off draw_sum
    }
    else
    {
      draw_sum = 1;
    }
    if (draw_diff == 1)
        {
          draw_diff = 0;  // turn-off difference plot
        }
    } 
    else if (key == '[')  // double integration time
  {
      inttime_count = inttime_count + 1;
      intmult = pow(2,inttime_count);
    myPort.write(91);
        if (draw_diff == 1)
        {
          draw_diff = 0;  // turn-off difference plot
        }
  }
  else if (key == ']')  // halve the integration time
  {
      inttime_count = inttime_count - 1;
      intmult = pow(2,inttime_count);
    myPort.write(93);
        if (draw_diff == 1)
        {
          draw_diff = 0;  // turn-off difference plot
        }
}  
  else if (key == 'L')  // Toggle Laser ON-OFF
  // Sony SLD3134VL  20mW violet blue laser diode 405nm
  {
    myPort.write(76);
    if (LSR_flag == 1)
    {
        LSR_flag = 0;
    }
    else
    {
        LSR_flag = 1;
    }
  }  
  else if (key == 'W')  //  Toggle White LED ON-OFF
  // Samsung LED Lighting LM561B White Neutral 4000K 2.95V 65mA 120° 4-SMD
  {
    myPort.write(87);
    if (LED_flag == 1)
    {
        LED_flag = 0;
    }
    else
    {
        LED_flag = 1;
    }
  } 
  else if (key == 'x' ) // Save;  turns off LSR and LED and exit
  {
    output.flush();  // Writes the remaining data to the file
    output.close();  // Finishes the file
    println();
    println("File Stored;  exiting.... ");  
    myPort.write(120);  // Turns off LED and LASER OFF regardless of state
    exit();  // Stops the program and data writing to file
  }
}

// void mousePressed() {
//  myPort.write(120);  // Turns off LED and LASER OFF and exists
//  exit(); 
//  }
