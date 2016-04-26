package eecs393team.eecs3933dapp;

import android.app.Instrumentation;
import android.test.ActivityInstrumentationTestCase2;
import android.test.TouchUtils;
import android.test.suitebuilder.annotation.SmallTest;
import android.widget.Button;
import android.widget.ImageButton;
import android.widget.ImageView;
import android.widget.ToggleButton;

/**
 * Created by austin on 10/29/15.
 */

public class STLViewActivityTest extends ActivityInstrumentationTestCase2<STLViewActivity> {

    STLViewActivity mSTLViewAct;
    ImageButton preferences;
    ImageButton loadButton;
    ToggleButton rotateButon;

    public STLViewActivityTest() {
        super(STLViewActivity.class);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        mSTLViewAct= getActivity();
        preferences = (ImageButton) getActivity().findViewById(R.id.preferencesButton);
        loadButton = (ImageButton) getActivity().findViewById(R.id.loadButton);
        rotateButon = (ToggleButton) getActivity().findViewById(R.id.rotateOrMoveToggleButton);
    }

    @SmallTest
    public void testPreconditions() {
        System.out.println("Testing Preconditions...");
        assertNotNull("mCreateMain is null", mSTLViewAct);
        assertEquals("Activity is of wrong type",
                STLViewActivity.class, mSTLViewAct.getClass());
        System.out.println("Done.");
    }

    public void testPreferences(){
        Instrumentation.ActivityMonitor prefMonitor =
                getInstrumentation().addMonitor(PreferencesActivity.class.getName(),
                        null, false);
        TouchUtils.clickView(this, preferences);
        PreferencesActivity prefActivity = (PreferencesActivity)
                prefMonitor.waitForActivityWithTimeout(1000);
        assertNotNull("PreferencesActivity is null", prefActivity);
        assertEquals("Monitor for PreferencesActivity has not been called", 1, prefMonitor.getHits());
        assertEquals("Activity is of wrong type",
                PreferencesActivity.class, prefActivity.getClass());
        getInstrumentation().removeMonitor(prefMonitor);
        prefActivity.finish();
    }
}
