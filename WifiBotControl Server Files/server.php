<?php

    // Load JSON state
    $string = file_get_contents("robotStateNew.json");
    $json_a= json_decode($string,true);

    // Send command to robot
    foreach ($json_a as $key => $val){
        echo $val;
        echo ",";
    }

?>

