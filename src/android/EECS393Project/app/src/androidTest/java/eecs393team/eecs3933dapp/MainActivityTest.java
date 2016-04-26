package eecs393team.eecs3933dapp;

import android.app.Instrumentation;
import android.content.Intent;
import android.test.ActivityInstrumentationTestCase2;
import android.test.ActivityUnitTestCase;
import android.test.TouchUtils;
import android.test.suitebuilder.annotation.MediumTest;
import android.widget.Button;

/**
 * Created by austin on 10/24/15.
 */
public class MainActivityTest extends ActivityInstrumentationTestCase2<MainActivity> {

    MainActivity mMainActivity;
    Button create_Button;
    Button edit_Button;
    Button print_Button;

    public MainActivityTest() {
        super(MainActivity.class);
    }

    @Override
    protected void setUp() throws Exception {
        super.setUp();
        mMainActivity = getActivity();
        //create_Button = (Button) getActivity().findViewById(R.id.create_button);
        edit_Button = (Button) getActivity().findViewById(R.id.edit_button);
    }

    public void testPreconditions() {
        System.out.println("Testing Preconditions...");
        assertNotNull("mMainActivity is null", mMainActivity);
        assertEquals("Activity is of wrong type",
                MainActivity.class, mMainActivity.getClass());
        System.out.println("Done.");
    }
    /*
    @MediumTest
    public void testLaunchCreate(){
        System.out.println("Starting Create test...");
        Instrumentation.ActivityMonitor createMainMonitor =
                getInstrumentation().addMonitor(CreateMain.class.getName(),
                        null, false);
        TouchUtils.clickView(this, create_Button);
        CreateMain createMainActivity = (CreateMain)
                createMainMonitor.waitForActivityWithTimeout(1000);

        assertNotNull("CreateMain is null", createMainActivity);
        assertEquals("Monitor for MainActivity has not been called", 1, createMainMonitor.getHits());
        assertEquals("Activity is of wrong type",
                CreateMain.class, createMainActivity.getClass());
        getInstrumentation().removeMonitor(createMainMonitor);
        System.out.println("Finished Create test.");
        createMainActivity.finish();
    } */

    @MediumTest
    public void testLaunchGallery(){
        Instrumentation.ActivityMonitor STLViewMonitor =
                getInstrumentation().addMonitor(Gallery.class.getName(),
                        null, false);
        TouchUtils.clickView(this, edit_Button);
        Gallery stlViewActivity = (Gallery)
                STLViewMonitor.waitForActivityWithTimeout(1000);

        assertNotNull("STLViewActivity is null", stlViewActivity);
        assertEquals("Monitor for STLViewActivity has not been called", 1, STLViewMonitor.getHits());
        assertEquals("Activity is of wrong type",
                Gallery.class, stlViewActivity.getClass());
        getInstrumentation().removeMonitor(STLViewMonitor);
        stlViewActivity.finish();
    }


    @Override
    public void tearDown() throws Exception{
        super.tearDown();
    }
}
