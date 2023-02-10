package frc.robot;

// Imports
import java.util.Collection;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Vector;
import edu.wpi.first.wpilibj.Preferences;

public class Pref {
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    private static Collection<String> v;
    private static String tempString;
    private static double tempDouble;

    public static HashMap<String, Double> prefDict = new HashMap<>();

    static {
        prefDict.put("SwerveTune", 0.);

        prefDict.put("SwerveTurnPoskP", .8);
        prefDict.put("SwerveTurnPoskI", .2);
        prefDict.put("SwerveTurnPoskD", .1);
        prefDict.put("SwerveTurnPoskIz", 1.);

        prefDict.put("SwerveTurnSMPoskP", .1);
        prefDict.put("SwerveTurnSMPoskI", 0.);
        prefDict.put("SwerveTurnSMPoskD", 0.);
        prefDict.put("SwerveTurnSMPoskIz", 0.);
        
    }

    public static void ensureRioPrefs() {
        // init();
        deleteUnused();
        addMissing();
    }

    public static void deleteUnused() {
        v = new Vector<String>();
        v = Preferences.getKeys();
        // v = (Vector<String>) RobotContainer.prefs.getKeys();
        String[] myArray = v.toArray(new String[v.size()]);

        for (int i = 0; i < v.size(); i++) {
        boolean doNotDelete = myArray[i].equals(".type");

        if (!doNotDelete && !prefDict.containsKey(myArray[i]) && Preferences.containsKey(myArray[i])) {
            Preferences.remove(myArray[i]);
        }
        }
    }

    public static void addMissing() {

        Iterator<Map.Entry<String, Double>> it = prefDict.entrySet().iterator();
        while (it.hasNext()) {
        Map.Entry<String, Double> pair = it.next();
        tempString = pair.getKey();
        tempDouble = pair.getValue();
        if (!Preferences.containsKey((tempString)))
            Preferences.setDouble(tempString, tempDouble);
        }
    }

    public static double getPref(String key) {
        if (prefDict.containsKey(key))
        return Preferences.getDouble(key, prefDict.get(key));
        else
        return 0;
    }

    public static void deleteAllPrefs(Preferences Preferences) {
        edu.wpi.first.wpilibj.Preferences.removeAll();
    }
}
