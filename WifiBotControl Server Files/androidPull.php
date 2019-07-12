<?php
//used by the Android device to request data from the json file.
    // Load JSON state
    $string = file_get_contents("arduinoPushResults.json");
    $json_a= json_decode($string,true);

    // get the data written to the json file and return it to the Web object so it can be parsed by the Android device.
    foreach ($json_a as $key => $val){
        echo $val;
        echo ",";
    }

?>

