// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.lang.Math;

public class LED extends SubsystemBase {
    private static AddressableLED m_led;
    private static AddressableLEDBuffer m_ledBuffer;
    private static AddressableLEDBuffer Buffer;
    private final Timer timer = new Timer();

    private static LED mInstance = null;
    
    // State variables for sectionCharge animation
    private int chargeState = 0;
    private int ledIndex = 0;
    private int blinkCount = 0;
    private double lastStateChangeTime = 0;

    public static synchronized LED getInstance() {
        if (mInstance == null) {
            mInstance = new LED();
        }
        return mInstance;
    }

    /** Creates a new LED. */
    public LED() {
        m_led = new AddressableLED(0);
        m_ledBuffer = new AddressableLEDBuffer(54);
        Buffer = new AddressableLEDBuffer(54);
        m_led.setLength(m_ledBuffer.getLength());
        m_led.setData(m_ledBuffer);
        m_led.start();
    }

    public void resetLED(int port, int length) {
        m_led.stop();
        m_led = new AddressableLED(port);
        m_ledBuffer = new AddressableLEDBuffer(length);
        m_led.setLength(m_ledBuffer.getLength());
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
        m_led.start();
        
    }

    int counter = 0;

    public void color(int r, int g, int b) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }

    public void nocolor() {
        color(0, 0, 0);
    }

    public void blink(int r, int g, int b) {
        timer.start();
        if (timer.get() < 0.1) {
            color(r, g, b);
        } else if (timer.get() < 0.2) {
            color(0, 0, 0);
        } else {
            timer.reset();

        }
    }



    // Improved marquee method with customizable frequency and speed
    public void marquee(int r, int g, int b, int frequency, double speed) {
        timer.start();
        // Lower speed value means faster animation
        double animationTime = 0.2 / speed;
        
        if (timer.get() < animationTime/2) {
            for (int i = 0; i < m_ledBuffer.getLength() - 1; i++) {
                if (((int) ((i + counter) / frequency)) % 2 == 0) {
                    m_ledBuffer.setRGB(i, r, g, b);
                } else {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
        } else if (timer.get() > animationTime) {
            counter++;
            timer.restart();
        }

        m_led.setData(m_ledBuffer);
    }
    
    // Maintain backward compatibility with original method
    public void marquee(int r, int g, int b) {
        // Default frequency of 6 and speed of 1.0
        marquee(r, g, b, 6, 1.0);
    }

    public void rainbowmarquee() {
        timer.start();
        if (timer.get() < 0.1) {
            for (int i = 0; i < m_ledBuffer.getLength() - 1; i++) {
                if ((i + counter) % 7 == 0){
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                }
                if ((i + counter) % 7 == 1){
                    m_ledBuffer.setRGB(i, 255, 100, 0);
                }
                if ((i + counter) % 7 == 2){
                    m_ledBuffer.setRGB(i, 255, 255, 0);
                    }
                if ((i + counter) % 7 == 3) {
                    m_ledBuffer.setRGB(i, 0, 255, 0);
                }
                if ((i + counter) % 7 == 4) {
                    m_ledBuffer.setRGB(i, 0, 127, 255);
                }
                if ((i + counter) % 7 == 5) {
                    m_ledBuffer.setRGB(i, 0, 0, 255);
                }
                if ((i + counter) % 7 == 6) {
                    m_ledBuffer.setRGB(i, 139, 0, 255);
                }
            }
        } else if (timer.get() < 0.2) {
        } else {
            counter++;
            timer.restart();
        }
        m_led.setData(m_ledBuffer);
    }

    public void rainbowblink() {
        timer.start();
        if (timer.get() < 0.1) {
            color(255, 0, 0);
        } else if (timer.get() < 0.2) {
            color(255, 100, 0);
        } else if (timer.get() < 0.3) {
            color(255, 255, 0);
        } else if (timer.get() < 0.4) {
            color(0, 255, 0);
        } else if (timer.get() < 0.5) {
            color(0, 127, 255);
        } else if (timer.get() < 0.6) {
            color(0, 0, 255);
        } else if (timer.get() < 0.7) {
            color(139, 0, 255);
        } else if (timer.get() < 0.8) {
            timer.reset();

        }
    }

    public void charge(int r,int g,int b,int blink) {
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
            m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        for (int j = 0; j < blink; j++) {
          for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
          }
          m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
          for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
          }
          m_led.setData(m_ledBuffer);
          try {
            Thread.sleep(100);
          } catch (InterruptedException e) {
            e.printStackTrace();
          }
        }
        for (int i = 0; i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        m_led.setData(m_ledBuffer);
      }
      
    public void chris() {
        timer.start();
        if (timer.get() < 0.1) {
            for (int i = 89; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 255, 255, 0);
            }
            for (int i = 0; i < m_ledBuffer.getLength() - 16; i++) {
                if ((i + counter) % 7 == 0) {
                    m_ledBuffer.setRGB(i, 255, 0, 0);
                }
                if ((i + counter) % 7 == 1) {
                    m_ledBuffer.setRGB(i, 255, 100, 0);
                }
                if ((i + counter) % 7 == 2) {
                    m_ledBuffer.setRGB(i, 255, 255, 0);
                }
                if ((i + counter) % 7 == 3) {
                    m_ledBuffer.setRGB(i, 0, 255, 0);
                }
                if ((i + counter) % 7 == 4) {
                    m_ledBuffer.setRGB(i, 0, 127, 255);
                }
                if ((i + counter) % 7 == 5) {
                    m_ledBuffer.setRGB(i, 0, 0, 255);
                }
                if ((i + counter) % 7 == 6) {
                    m_ledBuffer.setRGB(i, 139, 0, 255);
                }
            }
        } else if (timer.get() < 0.2) {
            for (int i = 89; i < m_ledBuffer.getLength(); i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        } else {
            counter++;
            timer.restart();
        }
        m_led.setData(m_ledBuffer);
    }

    public void breath(int  hue){
        timer.start();
        if (timer.get()<0.3){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 100);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<0.6){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 90);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<0.9){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 80);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<1.2){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 70);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<1.5){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 60);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<1.8){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 50);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<2.1){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 40);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<2.4){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 30);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<2.7){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 20);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<3){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 30);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<3.3){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 40);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<3.6){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 50);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<3.9){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 60);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<4.2){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 70);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<4.5){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 80);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<4.8){for(int i= 0;i<m_ledBuffer.getLength() -1;i++){
            m_ledBuffer.setHSV(i, hue, 255, 90);
        }
        m_led.setData(m_ledBuffer);
    }
        else if (timer.get()<5.1){timer.reset();}
      }
    @Override
    public void periodic() {

    }
    
    // Section control methods
    public void setSection(int startIndex, int length, int r, int g, int b) {
        for (int i = startIndex; i < startIndex + length && i < m_ledBuffer.getLength(); i++) {
            m_ledBuffer.setRGB(i, r, g, b);
        }
        m_led.setData(m_ledBuffer);
    }
    
    // First section: 0-11 (12 LEDs)
    public void setSection1(int r, int g, int b) {
        setSection(0, 12, r, g, b);
    }
    
    // Second section: 12-25 (14 LEDs)
    public void setSection2(int r, int g, int b) {
        setSection(12, 14, r, g, b);
    }
    
    // // Third section: 26-40 (15 LEDs)
    // public void setSection3(int r, int g, int b) {
    //     setSection(26, 15, r, g, b);
    // }
    
    // // Fourth section: 41-53 (13 LEDs)
    // public void setSection4(int r, int g, int b) {
    //     setSection(41, 13, r, g, b);
    // }
    
    // Set all sections with different colors
    public void setAllSections(int r1, int g1, int b1, int r2, int g2, int b2) {
        setSection1(r1, g1, b1);
        setSection2(r2, g2, b2);
        // setSection3(r3, g3, b3);
        // setSection4(r4, g4, b4);
    }
    
    // Charging effect for a specific section - non-blocking version using timer
    public void sectionCharge(int section, int r, int g, int b, int blink) {
        int startIndex = 0;
        int length = 0;
        
        // Determine section parameters
        switch(section) {
            case 1:
                startIndex = 0;
                length = 12;
                break;
            case 2:
                startIndex = 12;
                length = 14;
                break;
            default:
                return; // Invalid section
        }
        
        // First call initializes the animation
        if (chargeState == 0) {
            chargeState = 1;
            ledIndex = 0;
            blinkCount = 0;
            lastStateChangeTime = timer.get();
            
            // Reset section
            for (int i = startIndex; i < startIndex + length; i++) {
                m_ledBuffer.setRGB(i, 0, 0, 0);
            }
        }
        
        double currentTime = timer.get();
        double elapsedTime = currentTime - lastStateChangeTime;
        
        // State machine for charging animation
        switch (chargeState) {
            case 1: // Filling up LEDs one by one
                if (elapsedTime >= 0.05) { // 50ms between LEDs
                    if (ledIndex < length) {
                        m_ledBuffer.setRGB(startIndex + ledIndex, r, g, b);
                        ledIndex++;
                        lastStateChangeTime = currentTime;
                    } else {
                        chargeState = 2; // Move to blinking state
                        blinkCount = 0;
                        lastStateChangeTime = currentTime;
                    }
                }
                break;
                
            case 2: // Blinking the section
                if (blinkCount < blink * 2) {
                    if (elapsedTime >= 0.1) { // 100ms per blink state
                        if (blinkCount % 2 == 0) {
                            // Turn off
                            for (int i = startIndex; i < startIndex + length; i++) {
                                m_ledBuffer.setRGB(i, 0, 0, 0);
                            }
                        } else {
                            // Turn on
                            for (int i = startIndex; i < startIndex + length; i++) {
                                m_ledBuffer.setRGB(i, r, g, b);
                            }
                        }
                        blinkCount++;
                        lastStateChangeTime = currentTime;
                    }
                } else {
                    chargeState = 3; // Move to ending state
                    lastStateChangeTime = currentTime;
                }
                break;
                
            case 3: // Final state - turn off
                for (int i = startIndex; i < startIndex + length; i++) {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                }
                chargeState = 0; // Reset for next time
                break;
        }
        
        m_led.setData(m_ledBuffer);
    }
    
    /**
     * Show charging progress based on percentage in specified section
     * @param section Section number (1-4)
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param percentage Charging percentage (0.0-1.0)
     * @param blink Whether to blink when fully charged
     * @param fromRightToLeft Whether to fill from right-to-left instead of left-to-right
     */
    public void sectionChargePercentage(int section, int r, int g, int b, double percentage, boolean blink, boolean fromRightToLeft) {
        int startIndex = 0;
        int length = 0;
        
        // Clamp percentage between 0 and 1
        percentage = Math.max(0.0, Math.min(1.0, percentage));
        
        // Determine section parameters
        switch(section) {
            case 1:
                startIndex = 0;
                length = 12;
                break;
            case 2:
                startIndex = 12;
                length = 14;
                break;
            default:
                return; // Invalid section
        }
        
        // Calculate how many LEDs to light up
        int ledsToLight = (int)Math.ceil(percentage * length);
        
        // Reset section first
        for (int i = startIndex; i < startIndex + length; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        
        // Light up LEDs based on percentage
        if (fromRightToLeft) {
            // Fill from right to left
            for (int i = 0; i < ledsToLight; i++) {
                m_ledBuffer.setRGB(startIndex + length - 1 - i, r, g, b);
            }
        } else {
            // Fill from left to right
            for (int i = startIndex; i < startIndex + ledsToLight; i++) {
                m_ledBuffer.setRGB(i, r, g, b);
            }
        }
        
        // If fully charged and blink is true, use timer-based blinking instead of sleep
        if (percentage >= 0.99 && blink) {
            // Fast blinking effect based on timer
            double time = timer.get() % 0.4; // 400ms cycle
            if (time < 0.2) { // First half of cycle: lit
                // LEDs already set above
            } else { // Second half of cycle: off
                for (int i = startIndex; i < startIndex + length; i++) {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
        }
        
        // Only update LED data once
        m_led.setData(m_ledBuffer);
    }
    
    /**
     * Show charging progress based on percentage in specified section
     * (Backwards compatibility with old calls)
     */
    public void sectionChargePercentage(int section, int r, int g, int b, double percentage, boolean blink) {
        // For section 3, use right-to-left by default, left-to-right for all others
        boolean fromRightToLeft = (section == 3);
        sectionChargePercentage(section, r, g, b, percentage, blink, fromRightToLeft);
    }
    
    /**
     * Show height-based color in specified section
     * @param section Section number (1-2)
     * @param height Current height value
     * @param maxHeight Maximum height value for scaling
     * @param percentage Completion percentage (0.0-1.0)
     * @param blink Whether to blink when at target
     * @param fromRightToLeft Whether to fill from right-to-left instead of left-to-right
     */
    public void sectionHeightColor(int section, double height, double maxHeight, double percentage, boolean blink, boolean fromRightToLeft) {
        int startIndex = 0;
        int length = 0;
        
        // Clamp percentage between 0 and 1
        percentage = Math.max(0.0, Math.min(1.0, percentage));
        
        // Determine section parameters
        switch(section) {
            case 1:
                startIndex = 0;
                length = 12;
                break;
            case 2:
                startIndex = 12;
                length = 14;
                break;
            default:
                return; // Invalid section
        }
        
        // Calculate how many LEDs to light up
        int ledsToLight = (int)Math.ceil(percentage * length);
        
        // Reset section first
        for (int i = startIndex; i < startIndex + length; i++) {
            m_ledBuffer.setRGB(i, 0, 0, 0);
        }
        
        // Map height to HSV hue value (0-180)
        // Lower heights: red/orange (0-30), middle heights: green/cyan (60-120), upper heights: blue/purple (120-180)
        int hue = (int)(180.0 * (height / maxHeight));
        
        // Clamp hue value
        hue = Math.max(0, Math.min(180, hue));
        
        // Light up LEDs based on percentage
        if (fromRightToLeft) {
            // Fill from right to left
            for (int i = 0; i < ledsToLight; i++) {
                // Use HSV for nice color gradient based on height
                m_ledBuffer.setHSV(startIndex + length - 1 - i, hue, 255, 255);
            }
        } else {
            // Fill from left to right
            for (int i = startIndex; i < startIndex + ledsToLight; i++) {
                // Use HSV for nice color gradient based on height
                m_ledBuffer.setHSV(i, hue, 255, 255);
            }
        }
        
        // If at target and blink is true, use timer-based blinking
        if (percentage >= 0.99 && blink) {
            // Fast blinking effect based on timer
            double time = timer.get() % 0.4; // 400ms cycle
            if (time >= 0.2) { // Second half of cycle: off
                for (int i = startIndex; i < startIndex + length; i++) {
                    m_ledBuffer.setRGB(i, 0, 0, 0);
                }
            }
        }
        
        // Only update LED data once
        m_led.setData(m_ledBuffer);
    }
    
    /**
     * Blinks section1 (LEDs 0-11) with customizable color and speed
     * @param r Red value (0-255)
     * @param g Green value (0-255)
     * @param b Blue value (0-255)
     * @param blinkSpeed Blink speed multiplier (higher = faster)
     */
    public void blinkSection1(int r, int g, int b, double blinkSpeed) {
        timer.start();
        // Calculate blink cycle duration based on speed (0.6 seconds by default)
        double cycleDuration = 0.6 / blinkSpeed;
        double halfCycle = cycleDuration / 2;
        
        // Determine if we're in the "on" or "off" part of the cycle
        double cycleTime = timer.get() % cycleDuration;
        
        // First half of cycle: LEDs on
        if (cycleTime < halfCycle) {
            setSection1(r, g, b);
        } else {
            // Second half of cycle: LEDs off
            setSection1(0, 0, 0);
        }
    }
}