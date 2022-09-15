package org.firstinspires.ftc.teamcode.vision;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * A pipeline to attempt to find the three barcode red or blue dots on the field
 * in front of the robot
 *
 * Started with the FTCLib example code for UGBasicHighCoalPipeline,
 * because that detects red and blue rectangles...
 *
 */

public class BarcodeDetectionPipeline extends OpenCvPipeline {

    // which vertical percentage of the image to analyze
    private static final int VIEW_STARTING_Y_PERCENT = 70;
    private static final int VIEW_ENDING_Y_PERCENT = 100;

    // A copy of the telemetry object to sending messages
    // These messages can be seen in the EOVCSim program that
    // helps for testing the pipeline

    private Telemetry telemetry;

    // Stores the X,Y of the center of the image
    // and the minimum and maximum Y to be analyzed
    protected double centerX;
    protected double centerY;
    protected int minY, maxY;
    protected int imageWidth, imageHeight;

    // public variables to control how image is detected
    // These can be adjusted in EOCVSim
    //
    // Minimum bc square area Was 400, but blue square next to barrier can appear
    // smaller because partially hidden
    //
    // SU minimum area was 1000 and minimum sides 6, but when team cargo piece is placed on
    // square beside SU, it blocks some of SU tape so it appears as two pieces
    // so changed SU minimum area to 900 and minimum sides to 4, to allow a smaller part
    // of the SU tape to be allowed

    public int minThreshold = 145;   // Minimum level of red/blue to accept (0-255)
    public int maxThreshold = 220;   // Maximum level of red/blue to accept (0-255)
    public int minBCArea = 200;     // Minimum area of pixels accepted for a barcode square
    public int  maxBCArea = 800;    // Maximum area of pixels for a barcode square
    public int maxBCSides = 5;  // Maximum sides allow for a barcode square
    public int minSUArea = 900;    // Minimum area of pixel for piece of storage unit
    public int maxSUArea = 7000;   // Maximum area of pixels for a piece of storage unit
    public int minSUSides = 4;    // Minimum sides for a piece of storage unit
    public int maxSUSides = 12;     // Maximum sides for a piece of storage unti

    // These variables hold different version of the image from the camera
    // as they are processed

    private Mat blueThreshold;  // Image of just blue areas within the blue threshold
    private Mat redThreshold;   // Image of just red areas within the red threshold

    private Mat matYCrCb;       // The main image converted to YCrCb to get red and blue parts
    private Mat redChannel;     // The image with just the red part
    private Mat blueChannel;    // The image with just the blue part

    private List<MatOfPoint> redContours;   // A list of all the red shapes (contours) found
    private List<MatOfPoint> blueContours;  // A list of all the blue shapes (contours) found

    // The results of the image checking are stored here
    private int duckPosition = 0;   // The position of the duck 1, 2 or 3. 0 means cannot detect barcodes
    private boolean storageUnitVisible = false; // If true, barcode is beside storage unit

    /**
     * Set up the class
     *
     * @param telemetry
     */
    public BarcodeDetectionPipeline(Telemetry telemetry) {
        this.telemetry = telemetry;

        // Create all the Mat() object to hold the different images
        matYCrCb = new Mat();
        redChannel = new Mat();
        blueChannel = new Mat();

        blueThreshold = new Mat();
        redThreshold = new Mat();

        // Create the lists for holding the contours
        blueContours = new ArrayList<MatOfPoint>();
        redContours = new ArrayList<MatOfPoint>();

        // Set the threshold and area values
      //  minThreshold = 150;
      //  maxThreshold = 220;

 //       minBCArea = 200;    // Was 400, but blue square at barrier barrier appears smaller!
 //       maxBCArea = 800;

        storageUnitVisible = false;
    }

    /**
     * Get the size of the image we will be analyzing, and work out the center X,Y
     * and starting and ending Y of the area to be examined
     * @param mat
     */
    @Override
    public void init(Mat mat) {

        super.init(mat);
        imageWidth = mat.width();
        imageHeight = mat.height();

        centerX = ((double) imageWidth / 2) - 0.5;
        centerY = ((double) imageHeight / 2) - 0.5;

        minY = imageHeight * VIEW_STARTING_Y_PERCENT / 100;
        maxY = imageHeight * VIEW_ENDING_Y_PERCENT / 100;

    }


    /**
     * Check one shape (contour) to see if it is a barcode square or a piece of the storage unit.
     *
     * @param contour
     * @return  True if the contour seems to be a barcode square
     */
    public boolean filterContours(MatOfPoint contour) {
        double a = 0;

        // Sometimes the shape isn't exactly a square and it might have too many sides so
        // the approxPolyDP function is used to simplify the shape to a polygon that is almost
        // the same. This makes the barcode shape usually be exactly 4 sided, or sometimes 5
        // sided but never really any more than that

        MatOfPoint2f poly = new MatOfPoint2f();
        Imgproc.approxPolyDP(new MatOfPoint2f(contour.toArray()), poly, 3, true);

        // Now we know how many sides the shape has, and we work out how big it is
        // by getting the area in pixels
        int sideCount = poly.toArray().length;
        int area = (int)Imgproc.contourArea(contour);

        // These lines can be uncommented when debugging
        // Log.i("PIPELINE", "contour area="+area+", sides=" + sideCount);
        // telemetry.addData("Shape", "area="+area+", sides=" + sideCount);

        // If the number of sides and the area are within the range for a piece
        // of the storage unit, then set the variable to true
        if ( (sideCount >= minSUSides)
              && (sideCount <= maxSUSides)
              && (area > minSUArea)
              && (area < maxSUArea)
        ) {
            storageUnitVisible = true;
        }

        // If shape is right as edge of picture, it must be part of SU
        // and not a barcode, even if piece of SU that is visible is within the
        // shape parameters for a barcode square

        Rect r = Imgproc.boundingRect(contour);

        if ( ( r.x < 2)   // shape is against left side
             || ( (r.x + r.width) > (imageWidth - 2) ) )  // or shape against right side
            return false;

        // Check if the shape has the right amount of sides for a barcode square
        if (sideCount > 3 && sideCount <= maxBCSides ) {

            // Uncomment this for debugging to draw the contour we are
            // examining onto the image
                       // List<MatOfPoint> polyPoints = new ArrayList<>();
                       // polyPoints.add(new MatOfPoint(poly.toArray() ) );
                       // Imgproc.drawContours(input, polyPoints, -1, new Scalar(255, 0, 0), 1);


            // And if there are the right amount of sides, then
            // accept the item as a barcode square if the area
            // is also within range

            // Return true is the area is within range, or false if not
            return (minBCArea < area) && (area < maxBCArea);

        }

        // If we get this far, the contour was not a barcode square, so return false.
        return false;
    }

    /**
     * This is the main pipeline function that checks the camera image
     *
     * @param input  The camera image as a Mat object from OpenCV
     *
     * @return       The image, with any changes made by this pipeline
     *               changed or added to by this pipeline
     */
    @Override
    public Mat processFrame(Mat input) {

            // Convert image format to YCrCb format to make it easier
            // to extract red and blue channels

            Imgproc.cvtColor(input, matYCrCb, Imgproc.COLOR_RGB2YCrCb);

            // Make a rectangle for the area we are not interested in, and blank it
            Rect r = new Rect();
            r.x = 0;
            r.y = 0;
            r.width = matYCrCb.width();
            r.height = minY;

            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);

            r.y = maxY;
            r.height = matYCrCb.height() - maxY;
            Imgproc.rectangle(matYCrCb, r, new Scalar(0, 0, 0), Imgproc.FILLED);

            Core.extractChannel(matYCrCb, redChannel, 1);
            Core.extractChannel(matYCrCb, blueChannel, 2);


            // Filter the blue channel to only blue levels within  threshold
            Imgproc.threshold(blueChannel, blueThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
            // And same for Red..
            Imgproc.threshold(redChannel, redThreshold, minThreshold, maxThreshold, Imgproc.THRESH_BINARY);
       // if (true) return blueThreshold;


        // Make contour objects for red and blue
            blueContours.clear();
            redContours.clear();

            // Find the contours around red and blue areas filtered out previously

            Imgproc.findContours(blueThreshold, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            Imgproc.findContours(redThreshold, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));
        

        // if a shape that matches a piece of the storage unit is found, the following
            // will be changed to true during the filter process
            storageUnitVisible = false;

            blueContours = blueContours.stream().filter(i -> filterContours(i)).collect(Collectors.toList());

            // same for red..
            redContours = redContours.stream().filter(i -> filterContours(i)).collect(Collectors.toList());

//            Mat x = new Mat();
//        Imgproc.cvtColor(blueThreshold, x, Imgproc.COLOR_GRAY2RGB);
//        Imgproc.drawContours(x, blueContours, -1, new Scalar(255, 0, 255), 2);
//        if (true) return x;


//        redContours = redContours.stream().filter(i -> {
//            boolean appropriateAspect = ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height > 1)
            //                   && ((double) Imgproc.boundingRect(i).width / Imgproc.boundingRect(i).height < 2);
            //           return filterContours(i) && appropriateAspect;
            //       }).collect(Collectors.toList());

            // draw the outline of the remaining contours onto the image
            Imgproc.drawContours(input, redContours, -1, new Scalar(255, 255, 0));
            Imgproc.drawContours(input, blueContours, -1, new Scalar(255, 255, 0));

            // determine which barcodes are present
            List<MatOfPoint> barcodes = (blueContours.isEmpty() ? redContours : blueContours);

            // draw the bounding rectangles on each
           for(MatOfPoint b : barcodes) {
                Imgproc.rectangle(input,  Imgproc.boundingRect(b), new Scalar(0,255,255) );
            }

            // Draw where the X limits are
            int x1 = (int)(centerX * 0.6);
            int x2 = (int)(centerX * 1.2);

            Imgproc.line(input, new Point(x1, minY), new Point(x1, maxY), new Scalar(0,255,0) );
        Imgproc.line(input, new Point(x2, minY), new Point(x2, maxY), new Scalar(0,255,0) );

        boolean haveLeft = barcodes.stream().anyMatch(i -> Imgproc.boundingRect(i).x < x1);
            boolean haveRight = barcodes.stream().anyMatch(i -> Imgproc.boundingRect(i).x > x2);

            // figure out duck position 1, 2 or 3..
            // blue square nearing storage unit is level 3, furthest is level 1
            // red square nearest storage unit is level 1, furthest is level 3

        duckPosition = (isBlueVisible() || isRedVisible()) ? (haveLeft ? (haveRight ? 2 : 3) : 1) : 0;
            telemetry.addData("Duck Position", duckPosition);

/*
        if (!blueContours.isEmpty()) {

            boolean haveLeft = blueContours.stream().anyMatch( i -> Imgproc.boundingRect(i).x < (centerX * 0.7) );
            boolean haveRight = blueContours.stream().anyMatch( i -> Imgproc.boundingRect(i).x > (centerX * 1.3) );

            // figure out duck position 1, 2 or 3..

            duckPosition = haveLeft ? (haveRight ? 2 : 3) : 1;
            telemetry.addData("duckPos", duckPosition);

            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
//            biggestBlueContour = Collections.max(blueContours, (t0, t1) -> {
         //       return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            // });
//            blueRect = Imgproc.boundingRect(biggestBlueContour);
   //         Imgproc.rectangle(input, blueRect, new Scalar(0, 0, 255), 3);
        } else {
            blueRect = null;
        }

        // same for red

        if (!redContours.isEmpty()) {
            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            biggestRedContour = Collections.max(redContours, (t0, t1) -> {
                return Double.compare(Imgproc.boundingRect(t0).width, Imgproc.boundingRect(t1).width);
            });
            redRect = Imgproc.boundingRect(biggestRedContour);
            Imgproc.rectangle(input, redRect, new Scalar(255, 0, 0), 3);
        } else {
            redRect = null;
        }
*/
        telemetry.addData("Robot Position",
                (isStorageUnitVisible() ? "Storage Unit" : "Warehouse")
                + ", " + (isRedVisible() ? "Red" : "Blue") + " Side" );
            //telemetry.addData("Red visible", isRedVisible());
            //telemetry.addData("Blue visible", isBlueVisible());
            //telemetry.addData("Storage unit visible", isStorageUnitVisible());
            telemetry.update();

        r.y = minY;
        r.height = maxY - minY;
        Imgproc.rectangle(input, r, new Scalar(0, 255, 0));

        return input; //  blueThreshold; // blueChannel; //  matYCrCb; // input;
    }

    public int duckPos() {
        return duckPosition;
    }

    private int redCount() {
        return redContours == null ? 0 : redContours.size();
    }

    private int blueCount() {
        return blueContours == null ? 0 : blueContours.size();
    }

    public boolean isRedVisible() {
        return redCount() > blueCount();
      //  return (redContours != null && redContours.size() >= 1);
    }

    public boolean isBlueVisible()
    {
        // In case there is a red item in field, still assume as long as there is more blue
        return blueCount() > redCount();
    }

    public boolean isStorageUnitVisible() {
        return storageUnitVisible;
    }

}
