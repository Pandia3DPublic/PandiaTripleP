
export function getChart() {
    // Chart.defaults.global.defaultFontColor="#b6afaf";
    let chartVolume = new Chart(document.getElementById('id_LineChart'), {
        type: 'line',
        data: {
            labels: [],
            datasets: [{
                data: [],
                label: "Volume",
                backgroundColor: "#4144f2",
                borderColor: "#4144f2",
                fill: true,
            }
            ]
        },
        options: {
            animation: { duration: 250 },
            responsive: true,
            plugins: {
                title: {
                    display: true,
                    text: 'Volume over time',
                    font: {
                        size: 20
                    },
                    color: 'black'
                },
                legend: {
                    labels: {
                        color: 'black'
                    }
                }
            },
            scales: {
                y: {
                    ticks: {
                        color: 'black'
                    },
                    min: 0,
                    suggestedMax: 10,
                },
                x: {
                    ticks: {
                        color: 'black'
                    }
                }
            }
        },
    });
    return chartVolume;
}

