// static/script.js

$(document).ready(function() {

    $('#webcam-image').click(function(e) {
        var offset = $(this).offset();
        var width = $(this).width();
        var height = $(this).height();
        
        var x = e.pageX - offset.left;
        var y = e.pageY - offset.top;
    
        // Normalize the coordinates
        var x_normalized = x / width;
        var y_normalized = y / height;
    
        $.ajax({
            type: 'POST',
            url: '/get_coordinates',
            data: {'x': x_normalized, 'y': y_normalized},
            success: function(response) {
                console.log('Coordinates:', response);
            },
            error: function(xhr, status, error) {
                console.error('Error:', error);
            }
        });
    });
    


    // Function to send POST request to Flask server
    function updateAction(action) {
        $.ajax({
            type: 'POST',
            url: '/update_action',
            data: {action: action},
            success: function(response) {
                console.log('Action updated successfully');
            },
            error: function(error) {
                console.error('Error updating action:', error);
            }
        });
    }

    // Event listener for the "Generate Path" button
    $('#generate-path-btn').click(function() {
        updateAction('generate_path');
    });

    // Event listener for the "Solve" button
    $('#solve-btn').click(function() {
        updateAction('solve');
    });

    // Event listener for the "Cancel Run" button
    $('#cancel-btn').click(function() {
        updateAction('cancel_run');
    });
});
