# Extended Kalman Filter Project 
Hand-in for the project Extended Kalman Filter in Self-Driving Car Engineer Nanodegree Program

---

## Dependencies

I used Xcode 8.3 to as an IDE, folloing the instructions in the ide_profiles directory.

## To run the compiled code

The command to run the program is: `./ExtendedKF ../../data/sample-laser-radar-measurement-data-1.txt ../../data/sample-laser-radar-measurement-data-1-output.txt`

## RMSE

The RSME for the sample data was:
```
Accuracy - RMSE:
0.0786469
0.0819392
0.554527
0.556054
```



## Code 
I modified the code, using code snippets from the quizzes. I addition I had to use the formulas and comments from the lectures to add code for radar initialisation and update function.

## Additional tools
I verified the code using RSME and the kalman-tracker.py and the visualisation program supplied. I did not use the notebooks from `CarND-Mercedes-SF-Utilities`
