<?php
    // Arduino device will call to this URL and pass on data.  This file write to arduinoPushResults.json - which is read by the Android device
	// Load JSON state
    $string = file_get_contents("arduinoPushResults.json");
    $json_a= json_decode($string,true);

	//first part is the name of the value in the json text file that will be read by server.php when called from Arduino
	//second part is the URL value that is parsed.
    $json_a['result1'] = $_GET["URLresult1"];
    $json_a['result2'] = $_GET["URLresult2"];

    $fp = fopen('arduinoPushResults.json', 'w');
    fwrite($fp, json_encode($json_a));
    fclose($fp);

?>


