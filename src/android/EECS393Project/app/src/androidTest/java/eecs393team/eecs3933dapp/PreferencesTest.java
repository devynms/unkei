package eecs393team.eecs3933dapp;

import android.app.Instrumentation;
import android.test.ActivityInstrumentationTestCase2;
import android.test.TouchUtils;
import android.test.suitebuilder.annotation.SmallTest;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.SeekBar;
import android.widget.ToggleButton;

/**
 * Created by austin on 10/29/15.
 */
public class PreferencesTest extends ActivityInstrumentationTestCase2<PreferencesActivity> {

    PreferencesActivity mPref;
    SeekBar red;
    SeekBar blue;
    SeekBar green;
    SeekBar alpha;
    Button reset;
    ToggleButton axes;
    ToggleButton grids;

    public PreferencesTest(){
        super(PreferencesActivity.class);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        mPref= getActivity();
        red = (SeekBar) getActivity().findViewById(R.id.redSeekBar);
        green = (SeekBar) getActivity().findViewById(R.id.greenSeekBar);
        blue = (SeekBar) getActivity().findViewById(R.id.blueSeekBar);
        alpha = (SeekBar) getActivity().findViewById(R.id.alphaSeekBar);
        reset = (Button) getActivity().findViewById(R.id.resetButton);
        axes = (ToggleButton) getActivity().findViewById(R.id.axesToggleButton);
        grids = (ToggleButton) getActivity().findViewById(R.id.gridToggleButton);
    }

    @SmallTest
    public void testPreconditions() {
        System.out.println("Testing Preconditions...");
        assertNotNull("mPref is null", mPref);
        assertEquals("Activity is of wrong type",
                PreferencesActivity.class, mPref.getClass());
        System.out.println("Done.");
    }

    public void testResetButton(){
        red.setProgress(0);
        blue.setProgress(0);
        green.setProgress(0);
        TouchUtils.clickView(this, reset);
        assertEquals((int) (red.getMax() * .75f), red.getProgress());
        assertEquals((int)(red.getMax() * .75f), blue.getProgress());
        assertEquals((int) (red.getMax() * .75f), green.getProgress());
        assertEquals((int) (alpha.getMax() * .5f), alpha.getProgress());
        assertTrue(axes.isChecked());
        assertTrue(grids.isChecked());
    }

    public void testFinish(){
        Button close = (Button) getActivity().findViewById(R.id.closeButton);
        Instrumentation.ActivityMonitor stlMonitor =
                getInstrumentation().addMonitor(STLViewActivity.class.getName(),
                        null, false);
        TouchUtils.clickView(this, close);
        STLViewActivity stlActivity = (STLViewActivity)
                stlMonitor.waitForActivityWithTimeout(1000);
        assertNull("STLViewActivity is null", stlActivity);

        getInstrumentation().removeMonitor(stlMonitor);

    }

    public void testPause(){
        int redVal = red.getProgress();
        int greenVal = green.getProgress();
        int blueVal = blue.getProgress();
        int alphaVal = alpha.getProgress();
        Instrumentation ins = getInstrumentation();
        ins.callActivityOnPause(mPref);
        assertEquals((float) redVal / (float) red.getMax(), STLRenderer.red);
        assertEquals((float) greenVal / (float) green.getMax(),STLRenderer.green);
        assertEquals((float) blueVal / (float) blue.getMax(),STLRenderer.blue);
        assertEquals((float) alphaVal / (float) alpha.getMax(),STLRenderer.alpha);
    }

}
